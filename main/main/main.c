#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include <math.h>

// DIR_1 9
// STEP_1 10
// DIR_2 11
// STEP_2 12

// Pins
#define PIN_ENABLE_ALL 8
#define PIN_DIR_1 25
#define PIN_STEP_1 26
#define PIN_DIR_2 12
#define PIN_STEP_2 13
#define PIN_DIR_3 13
#define PIN_STEP_3 14
#define PIN_DIR_4 15
#define PIN_STEP_4 16
#define PIN_BUTTON 17
#define PIN_LED 18

// Motor characteristics
#define DEFAULT_DIRECTION 0
#define MOTOR_DIRECTION(direction) ((direction == DEFAULT_DIRECTION) ? 0 : 1)
#define MAX_DIST 1000 // steps
#define MAX_SPEED 20 // steps/s
#define MAX_ACCEL 1 // steps/s

// Task characteristics
#define MOTOR_TASK_STACK_SIZE 2048
#define MOTOR_TASK_PRIORITY (configMAX_PRIORITIES - 1)

typedef struct {

    int current_speed;

    int dir_pin;
    int step_pin;

    SemaphoreHandle_t mutex;

} Motor;


typedef struct {

    Motor* left_motor;
    Motor* right_motor;
    int current_pos;
    int target_pos;
    int current_speed;

} MotorPair;

static void wait(double seconds) {
    vTaskDelay((seconds * 1000) / portTICK_PERIOD_MS);
}

/**
 * Set speed of a motor
 * @param motor Pointer to the motor to set
 * @param speed Integer speed
*/
static void set_speed(MotorPair *motor_pair, int speed) {
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    }
    motor_pair->current_speed = speed;
}

/**
 * Run the motor. This function will run continuously and should be scheduled
 * with a xTaskCreate function call.
 * @param pvParameters Passed in from the xTaskCreate call. Pointer to a
 * MotorPair that this function will control.
*/
static void run_motors(void* pvParameters) {

    MotorPair* motors = (MotorPair*)pvParameters;
    Motor* left_motor = motors->left_motor;
    Motor* right_motor = motors->right_motor;

    bool has_lm_mutex = false;
    bool has_rm_mutex = false;

    TickType_t movement_start_ticks = -1;

    while(1) {

        BaseType_t lm_taken = pdFALSE;
        BaseType_t rm_taken = pdFALSE;

        // Take semaphores only if we don't already have them from last 
        // iteration
        if (!has_lm_mutex) {
            // Try to take the mutex
            lm_taken = xSemaphoreTake(left_motor->mutex, portMAX_DELAY);
            // Store whether we took the mutex
            has_lm_mutex = (lm_taken == pdTRUE) ? true: false;
        }
        if (!has_rm_mutex) {
            // Try to take the mutex
            rm_taken = xSemaphoreTake(right_motor->mutex, portMAX_DELAY);
            // Store whether we took the mutex
            has_rm_mutex = (rm_taken == pdTRUE) ? true: false;
        }
        // If getting the mutex failed, try again
        if (!has_lm_mutex || !has_rm_mutex) {
            continue;
        }


        // Check if motors are at the target position
        if (motors->current_pos == motors->target_pos)
        {
            // Equalized at target position, stop all motors
            set_speed(motors, 0);
            //printf("Motors stopped\n");

            // No movement, so set movement start time to -1
            movement_start_ticks = -1;
            
            // Give up mutex locks
            xSemaphoreGive(left_motor->mutex);
            xSemaphoreGive(right_motor->mutex);
            has_lm_mutex = false;
            has_rm_mutex = false;

            vTaskDelay(10 / portTICK_PERIOD_MS);

        } else {
            // Motors need to move

            int direction = 1;

            // Calculate steps required
            int steps_to_move = motors->target_pos - motors->current_pos;
            
            // Set direction
            if (steps_to_move < 0) {
                direction = 0;
                steps_to_move *= -1;
            }
            gpio_set_level(left_motor->dir_pin, MOTOR_DIRECTION(direction));
            gpio_set_level(right_motor->dir_pin, MOTOR_DIRECTION(direction));

            // Adjust speed based on acceleration
            if(movement_start_ticks == -1) {
                movement_start_ticks = xTaskGetTickCount();
                set_speed(motors, 1);
            } else {
                TickType_t current_ticks = xTaskGetTickCount();
                TickType_t elapsed_seconds = (double)(current_ticks - movement_start_ticks) / configTICK_RATE_HZ;
                
                if(motors->current_speed < MAX_SPEED) {
                    int speed_increment = round(MAX_ACCEL * elapsed_seconds);
                    set_speed(motors, motors->current_speed + speed_increment);
                }
            }

            // Move the motors
            gpio_set_level(left_motor->step_pin, 1);
            gpio_set_level(right_motor->step_pin, 1);
            vTaskDelay(1000 / motors->current_speed / 2 / portTICK_PERIOD_MS);
            gpio_set_level(left_motor->step_pin, 0);
            gpio_set_level(right_motor->step_pin, 0);
            vTaskDelay(1000 / motors->current_speed / 2 / portTICK_PERIOD_MS);

            if(direction == 1) {
                motors->current_pos += 1;
            } else {
                motors->current_pos -= 1;
            }
            
            printf("pos %d\n", motors->current_pos);

            // Give up mutex locks
            xSemaphoreGive(left_motor->mutex);
            xSemaphoreGive(right_motor->mutex);
            has_lm_mutex = false;
            has_rm_mutex = false;

        }
    }
}

int step_target = 0;

void app_main(void)
{

    gpio_config_t motor_1_config = {
        .pin_bit_mask = (1ULL << PIN_DIR_1) | (1ULL << PIN_STEP_1),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    gpio_config_t motor_2_config = {
        .pin_bit_mask = (1ULL << PIN_DIR_2) | (1ULL << PIN_STEP_2),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    gpio_config(&motor_1_config);
    gpio_config(&motor_2_config);


    Motor* blackout_left = malloc(sizeof(Motor));
    Motor* blackout_right = malloc(sizeof(Motor));
    MotorPair* blackout_pair = malloc(sizeof(MotorPair));

    // Check for memory allocation errors
    if (blackout_left == NULL || blackout_right == NULL || blackout_pair == NULL) {
        esp_restart();
        printf("Memory allocation error.\n");
        return;
    }

    blackout_left->mutex = xSemaphoreCreateMutex();
    blackout_left->dir_pin = PIN_DIR_1;
    blackout_left->step_pin = PIN_STEP_1;

    blackout_right->mutex = xSemaphoreCreateMutex();
    blackout_right->dir_pin = PIN_DIR_2;
    blackout_right->step_pin = PIN_STEP_2;

    blackout_pair->left_motor = blackout_left;
    blackout_pair->right_motor = blackout_right;
    blackout_pair->current_pos = 0;
    blackout_pair->target_pos = 0;
    blackout_pair->current_speed = 0;

    xTaskCreate(run_motors, "blackout_run_task", MOTOR_TASK_STACK_SIZE, (void*)blackout_pair, MOTOR_TASK_PRIORITY, NULL);

    wait(3);

    printf("Running to 100...\n");
    blackout_pair->target_pos = 100;
    wait(10);

    printf("Running to 20...\n");
    blackout_pair->target_pos = 20;
    wait(10);

    printf("Running to 33...\n");
    blackout_pair->target_pos = 33;
    wait(5);

}

