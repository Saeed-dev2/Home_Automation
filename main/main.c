/*
 * Home Automation Project
 * Proximity-Based Control System
 * 
 * This project implements a home automation system using an ESP32,
 * an ultrasonic sensor (HC-SR04), and an IR sensor. The system controls
 * an LED based on the distance measured by the ultrasonic sensor and
 * the detection of objects by the IR sensor.
 * 
 * Components:
 * - ESP32
 * - HC-SR04 Ultrasonic Sensor
 * - IR Sensor
 * - LED
 * 
 * Connections:
 * - HC-SR04:
 *   - VCC to 5V
 *   - GND to GND
 *   - Trig to GPIO 33
 *   - Echo to GPIO 32
 * - IR Sensor:
 *   - VCC to 3.3V
 *   - GND to GND
 *   - Signal to GPIO 26
 * - LED:
 *   - Anode to GPIO 27
 *   - Cathode to GND (with a current limiting resistor if necessary)
 * 
 * Author: M.Saeed
 * Date: July 2024
 * 
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_PIN         GPIO_NUM_27   // GPIO pin connected to LED
#define IR_SENSOR_PIN   GPIO_NUM_26   // GPIO pin connected to IR sensor
#define TRIGGER_PIN     GPIO_NUM_33   // GPIO pin connected to ultrasonic sensor trigger
#define ECHO_PIN        GPIO_NUM_32   // GPIO pin connected to ultrasonic sensor echo

static const char *TAG = "home_automation";

/**
 * @brief Initialize GPIO pins.
 */
void init_gpio() {
    gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(IR_SENSOR_PIN);
    gpio_set_direction(IR_SENSOR_PIN, GPIO_MODE_INPUT);

    gpio_pad_select_gpio(TRIGGER_PIN);
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);

    gpio_pad_select_gpio(ECHO_PIN);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
}

/**
 * @brief Task to control LED based on motion detection and distance measurement.
 * 
 * @param pvParameter Task parameter, not used in this project.
 */
void control_led_task(void *pvParameter) {
    while (1) {
        // Check IR sensor for motion detection
        if (gpio_get_level(IR_SENSOR_PIN) == 0) {
            ESP_LOGI(TAG, "Motion detected!");

            // Check distance using ultrasonic sensor
            gpio_set_level(TRIGGER_PIN, 1);
            ets_delay_us(10);
            gpio_set_level(TRIGGER_PIN, 0);

            // Measure distance
            int64_t start_time = esp_timer_get_time();
            int64_t end_time = start_time;
            while (gpio_get_level(ECHO_PIN) == 0 && (end_time - start_time < 50000)) {
                end_time = esp_timer_get_time();
            }
            start_time = end_time;
            while (gpio_get_level(ECHO_PIN) == 1 && (end_time - start_time < 50000)) {
                end_time = esp_timer_get_time();
            }

            // Calculate distance in cm
            int distance = (int)(((end_time - start_time) * 0.0343) / 2);

            ESP_LOGI(TAG, "Distance: %d cm", distance);

            // Control LED based on distance
            if (distance < 50) {
                gpio_set_level(LED_PIN, 1);  // Turn on LED
                ESP_LOGI(TAG, "LED turned on");
                vTaskDelay(1000 / portTICK_PERIOD_MS);  // Stay on for 1 second
            } else {
                gpio_set_level(LED_PIN, 0);  // Turn off LED
                ESP_LOGI(TAG, "LED turned off");
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);  // Check every 100ms
    }
}

/**
 * @brief Main function, entry point of the program.
 */
void app_main() {
    init_gpio();

    // Create task to control LED based on sensors
    // Freertos using to create Task
    xTaskCreate(control_led_task, "control_led_task", 4096, NULL, 5, NULL);
}
