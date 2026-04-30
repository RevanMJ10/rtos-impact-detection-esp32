#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ---------------- STATES ----------------
typedef enum {
    STATE_IDLE,
    STATE_IMPACT,
    STATE_STABLE
} MotionState;

MotionState currentState = STATE_IDLE;

// ---------------- I2C CONFIG ----------------
#define I2C_MASTER_SCL_IO     14
#define I2C_MASTER_SDA_IO     13
#define I2C_MASTER_NUM        I2C_NUM_0
#define I2C_MASTER_FREQ_HZ    400000

#define KX13X_ADDR   0x1E
#define BUZZER_PIN   25

// ---------------- REGISTERS ----------------
#define KX13X_WHO_AM_I   0x13
#define KX13X_INS2       0x17
#define KX13X_XOUT_L     0x08
#define KX13X_CNTL1      0x1B
#define KX13X_ODCNTL     0x21

#define CNTL1_STANDBY    0x00
#define CNTL1_OPERATE    0xE0

// ---------------- THRESHOLDS ----------------
#define IMPACT_THRESHOLD   3500.0f  // raised — avoids drop-release false trigger
#define STABLE_THRESHOLD   300.0f
#define STABLE_HOLD_MS     1000
#define IMPACT_LOCKOUT_MS  300      // ignore bounce spikes after first impact

// ---------------- QUEUES ----------------
QueueHandle_t sensorQueue;
QueueHandle_t alertQueue;

float base_x = 0, base_y = 0, base_z = 0;
static int impact_count = 0;

// ---------------- I2C ----------------
void i2c_master_init() {
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t write_register(uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (KX13X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, value, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t read_registers(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (KX13X_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (KX13X_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ---------------- SENSOR INIT ----------------
void sensor_init() {
    uint8_t who = 0;
    read_registers(KX13X_WHO_AM_I, &who, 1);
    printf("WHO_AM_I: 0x%02X (expect 0x3D or 0x46)\n", who);

    write_register(KX13X_CNTL1, CNTL1_STANDBY);
    vTaskDelay(pdMS_TO_TICKS(10));
    write_register(KX13X_ODCNTL, 0x06);
    write_register(KX13X_CNTL1, CNTL1_OPERATE);
    vTaskDelay(pdMS_TO_TICKS(100));

    printf("Sensor initialized.\n");
}

// ---------------- BUZZER ----------------
void buzzer_init() {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 2000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&channel);
}

void buzzer_on()  { ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512); ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); }
void buzzer_off() { ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);   ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0); }

// ---------------- CALIBRATION ----------------
void calibrate_sensor() {
    printf("Calibrating... keep sensor still.\n");

    uint8_t data[6];
    int samples = 50;

    for (int i = 0; i < samples; i++) {
        read_registers(KX13X_XOUT_L, data, 6);
        base_x += (int16_t)((data[1] << 8) | data[0]);
        base_y += (int16_t)((data[3] << 8) | data[2]);
        base_z += (int16_t)((data[5] << 8) | data[4]);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    base_x /= samples;
    base_y /= samples;
    base_z /= samples;

    printf("Baseline → X:%.2f Y:%.2f Z:%.2f\n", base_x, base_y, base_z);
}

// ---------------- SENSOR TASK ----------------
void sensorTask(void *pvParameters) {
    uint8_t data[6];

    while (1) {
        uint8_t status = 0;
        read_registers(KX13X_INS2, &status, 1);

        if (status & 0x10) {
            read_registers(KX13X_XOUT_L, data, 6);

            int16_t values[3] = {
                (int16_t)((data[1] << 8) | data[0]),
                (int16_t)((data[3] << 8) | data[2]),
                (int16_t)((data[5] << 8) | data[4])
            };

            xQueueSend(sensorQueue, values, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------- PROCESS TASK ----------------
void processTask(void *pvParameters) {
    int16_t values[3];
    static float prev_x = 0, prev_y = 0, prev_z = 0;
    static float max_delta = 0;
    static int last_alert = 0;
    static TickType_t stable_entry_time = 0;
    static TickType_t impact_time = 0;

    while (1) {
        if (xQueueReceive(sensorQueue, values, portMAX_DELAY)) {

            float x = values[0] - base_x;
            float y = values[1] - base_y;
            float z = values[2] - base_z;

            float dx = x - prev_x;
            float dy = y - prev_y;
            float dz = z - prev_z;
            float delta = sqrtf(dx*dx + dy*dy + dz*dz);

            prev_x = x; prev_y = y; prev_z = z;

            // ---------------- FSM ----------------
            switch (currentState) {

                case STATE_IDLE:
                    if (delta > IMPACT_THRESHOLD) {
                        currentState = STATE_IMPACT;
                        impact_count++;
                        impact_time = xTaskGetTickCount();
                        max_delta = delta;  // start tracking peak

                        printf(">>> IMPACT #%d detected | Delta:%.1f <<<\n",
                               impact_count, delta);
                    }
                    break;

                case STATE_IMPACT:
                    // track peak delta during entire impact phase
                    if (delta > max_delta)
                        max_delta = delta;

                    // wait out lockout period before checking for settle
                    if ((xTaskGetTickCount() - impact_time) > pdMS_TO_TICKS(IMPACT_LOCKOUT_MS)) {
                        if (delta < STABLE_THRESHOLD) {
                            currentState = STATE_STABLE;
                            stable_entry_time = xTaskGetTickCount();

                            // severity decided at peak — accurate
                            const char *severity;
                            if      (max_delta > 7000) severity = "SEVERE";
                            else if (max_delta > 5000) severity = "MODERATE";
                            else                       severity = "MINOR";

                            printf(">>> IMPACT #%d | %s | Peak Delta:%.1f <<<\n",
                                   impact_count, severity, max_delta);

                            max_delta = 0;  // reset for next impact
                        }
                    }
                    break;

                case STATE_STABLE:
                    if (delta > IMPACT_THRESHOLD) {
                        currentState = STATE_IMPACT;
                        impact_count++;
                        impact_time = xTaskGetTickCount();
                        max_delta = delta;  // start fresh for re-impact

                        printf(">>> IMPACT #%d detected | Re-impact | Delta:%.1f <<<\n",
                               impact_count, delta);
                    }
                    else if ((xTaskGetTickCount() - stable_entry_time) > pdMS_TO_TICKS(STABLE_HOLD_MS)) {
                        currentState = STATE_IDLE;
                        printf("--- Settled → IDLE ---\n");
                    }
                    break;
            }

            // DEBUG
            printf("Delta:%.1f | State:", delta);
            switch (currentState) {
                case STATE_IDLE:   printf("IDLE\n");   break;
                case STATE_IMPACT: printf("IMPACT\n"); break;
                case STATE_STABLE: printf("STABLE\n"); break;
            }

            // ALERT — send trigger only on new impact, never send 0
            if (currentState == STATE_IMPACT && last_alert != 3) {
                xQueueSend(alertQueue, &(int){3}, 0);
                last_alert = 3;
            } else if (currentState == STATE_IDLE && last_alert != 0) {
                last_alert = 0;
            }
        }
    }
}

// ---------------- ALERT TASK ----------------
void alertTask(void *pvParameters) {
    int alert;
    int impact_active = 0;
    TickType_t impact_start = 0;

    while (1) {
        if (xQueueReceive(alertQueue, &alert, pdMS_TO_TICKS(10))) {
            if (alert == 3) {
                // reset timer on every new impact — extends alarm if hit again
                impact_active = 1;
                impact_start = xTaskGetTickCount();
            }
        }

        if (impact_active) {
            buzzer_on();
            vTaskDelay(pdMS_TO_TICKS(200));
            buzzer_off();
            vTaskDelay(pdMS_TO_TICKS(200));

            // stop after 5 seconds from last impact
            if ((xTaskGetTickCount() - impact_start) > pdMS_TO_TICKS(5000)) {
                impact_active = 0;
                buzzer_off();
                printf("--- Alarm stopped ---\n");
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ---------------- MAIN ----------------
void app_main() {
    i2c_master_init();
    sensor_init();
    buzzer_init();
    calibrate_sensor();

    sensorQueue = xQueueCreate(5, sizeof(int16_t) * 3);
    alertQueue  = xQueueCreate(5, sizeof(int));

    xTaskCreate(sensorTask,  "Sensor",  4096, NULL, 3, NULL);
    xTaskCreate(processTask, "Process", 4096, NULL, 2, NULL);
    xTaskCreate(alertTask,   "Alert",   2048, NULL, 1, NULL);
}
