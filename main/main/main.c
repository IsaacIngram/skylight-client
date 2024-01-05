//
// main.c
// 
// Makeplus
// Isaac Ingram
//
// Control skylights using stepper motor control, with target values
// being received over a mesh ESP-NOW network. 
//
// Debugging Note: Use baudrate 115200
//

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include <math.h>
#include "app_state.h"


// PINOUT Definitions
#ifdef ON_TARGET

#define PIN_ENABLE_ALL 8
#define BO_LEFT_DIR_PIN 9
#define BO_LEFT_STEP_PIN 10
#define BO_RIGHT_DIR_PIN 11
#define BO_RIGHT_STEP_PIN 12 
#define F_LEFT_DIR_PIN 13
#define F_LEFT_STEP_PIN 14
#define F_RIGHT_DIR_PIN 15
#define F_RIGHT_STEP_PIN 16
#define BUTTON_PIN 17
#define LED_PIN 18   

#else

// Pins
#define PIN_ENABLE_ALL 8
#define BO_LEFT_DIR_PIN 25
#define BO_LEFT_STEP_PIN 26
#define BO_RIGHT_DIR_PIN 12
#define BO_RIGHT_STEP_PIN 13
#define F_LEFT_DIR_PIN 13
#define F_LEFT_STEP_PIN 14
#define F_RIGHT_DIR_PIN 15
#define F_RIGHT_STEP_PIN 16
#define BUTTON_PIN 17
#define LED_PIN 18

#endif //ON_TARGET

// Motor characteristics
#define DEFAULT_DIRECTION 0
#define MOTOR_DIRECTION(direction) ((direction == DEFAULT_DIRECTION) ? 0 : 1)
#define MAX_DIST 1000 // steps
#define MAX_SPEED 10 // steps/s
#define MAX_ACCEL 3 // steps/s
#define MAX_DECEL 3 // steps/s

// Task characteristics
#define MOTOR_TASK_STACK_SIZE 2048
#define MOTOR_TASK_PRIORITY (configMAX_PRIORITIES - 1)

AppStateContainer app_state;

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
    int direction;
} MotorPair;

/**
 * vTaskDelay a number of seconds
 * @param seconds Double seconds
*/
static void wait(double seconds) {
    vTaskDelay((seconds * 1000) / portTICK_PERIOD_MS);
}

/**
 * Initialize the app state variable. This function will automatically restart
 * the esp if there is an error creating the semaphore.
*/
void init_app_state() {
    app_state.state = STATE_BOOT;
    app_state.mutex = xSemaphoreCreateMutex();
    if(app_state.mutex == NULL) {
        printf("Error creating semaphore for state variable.\n");
        esp_restart();
    }
}

/**
 * Cleanup app state.
*/
void cleanup_app_state() {
    vSemaphoreDelete(app_state.mutex);
}

/**
 * Set speed of a motor
 * @param motor Pointer to the motor to set
 * @param speed Integer speed
*/
static void set_speed(MotorPair *motor_pair, int speed) {
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    } else if(speed < 0) {
        speed = 0;
    }
    motor_pair->current_speed = speed;
}

/**
 * Configure motor GPIO pins
 * @param step_pin Int step pin
 * @param dir_pin Int dir pin
 * 
*/
static void config_motor_pins(int step_pin, int dir_pin) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << dir_pin) | (1ULL << step_pin),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
}

/**
 * Set the direction pins for a pair of motors
 * @param direction Integer direction. 1 for forward, -1 for backward or 0 for backwards
*/
static void set_direction_pins(MotorPair *motorPair, int direction) {
    if(direction == 1) {
        gpio_set_level(motorPair->left_motor->dir_pin, MOTOR_DIRECTION(1));
        gpio_set_level(motorPair->right_motor->dir_pin, MOTOR_DIRECTION(1));
    } else if(direction == 0 || direction == -1) {
        gpio_set_level(motorPair->left_motor->dir_pin, MOTOR_DIRECTION(0));
        gpio_set_level(motorPair->right_motor->dir_pin, MOTOR_DIRECTION(0));
    }
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
    TickType_t decel_start_ticks = -1;

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
            printf("Mutex failure\n");
            continue;
        }


        // Check if motors are at the target position
        if (motors->current_pos == motors->target_pos)
        {
            // Equalized at target position, stop all motors
            set_speed(motors, 0);
            motors->direction = 0;

            // No movement, so set movement start time to -1
            movement_start_ticks = -1;
            
            // Give up mutex locks
            xSemaphoreGive(left_motor->mutex);
            xSemaphoreGive(right_motor->mutex);
            has_lm_mutex = false;
            has_rm_mutex = false;
            //printf("at target %d\n", motors->target_pos);

            vTaskDelay(10 / portTICK_PERIOD_MS);

        } else {
            // Motors need to move

            // Calculate steps and direction
            int target_direction = 1;
            int steps_to_move = motors->target_pos - motors->current_pos;
            if(steps_to_move < 0) {
                target_direction = -1;
                steps_to_move *= -1;
            }

            // Check if movement hasn't started yet
            if(movement_start_ticks == -1) {
                // Start at 1 tick/s in the target direction
                motors->direction = target_direction;
                motors->current_speed = 1;
                movement_start_ticks = xTaskGetTickCount();
                printf("New movement\n");
                continue;
            }

            // TickType_t is an unsigned 32 bit type
            TickType_t current_ticks = xTaskGetTickCount();
            double elapsed_seconds = ((double)current_ticks - (double)movement_start_ticks) / (double)configTICK_RATE_HZ;

            // Check if there is a change in direction
            int current_direction = motors->direction;
            if (current_direction == 0 || current_direction == target_direction) {
                // No change in direction

                // Calculate deceleration time based on current speed
                double time_to_decel = motors->current_speed / MAX_DECEL;
                double decel_distance = (motors->current_speed * motors->current_speed) / (2 * MAX_DECEL);
                printf("Decel time: %f, dist: %f\n", time_to_decel, decel_distance);
                // Check if deceleration is necessary to reach target
                if((int)decel_distance >= steps_to_move) {
                    // Need to decelerate
                    printf("Must decel\n");
                    if(decel_start_ticks == -1) {
                        decel_start_ticks = current_ticks;
                        int new_speed = motors->current_speed - MAX_DECEL;
                        set_speed(motors, new_speed);
                    } else {
                        double decel_elapsed = ((double)current_ticks - (double)decel_start_ticks) / (double)configTICK_RATE_HZ;
                        // Limit to 1 or above so motor starts slowing immediately
                        if(decel_elapsed < 1) {
                            decel_elapsed = 1;
                        }
                        int speed_delta = (int)(MAX_DECEL * decel_elapsed);
                        int new_speed = motors->current_speed - speed_delta;
                        set_speed(motors, new_speed);
                    }
                        
                    // int new_speed = (int)(MAX_DECEL * elapsed_seconds);
                    // set_speed(motors, new_speed);
                    // if((motors->current_speed - speed_increment) <= 0) {
                    //     motors->direction = 0;
                    // }
                } else if(motors->current_speed < MAX_SPEED) {
                    // Need to accelerate
                    printf("Must accel ");
                    int new_speed = (int)round(MAX_ACCEL * elapsed_seconds);
                    if(new_speed < 1) {
                        new_speed = 1;
                    }
                    set_speed(motors, new_speed);
                    printf("\tnew speed: %d\n", motors->current_speed);

                } else {
                    // Maintain current velocity
                    printf("Maintain current\n");
                }
            } else { 
                // Direction changed
                printf("Direction changed\n");
                int speed_increment = round(MAX_DECEL * elapsed_seconds);
                set_speed(motors, motors->current_speed - speed_increment);
                if((motors->current_speed - speed_increment <= 0)) {
                    motors->direction = -current_direction;
                }
            }

            set_direction_pins(motors, motors->direction);
            if(motors->direction == 1) {
                motors->current_pos += 1;
            } else if(motors->direction == -1) {
                motors->current_pos -=1;
            }

            if(motors->current_speed == 0) {
                // Don't need to move motors; reset everything
                movement_start_ticks = -1;
                decel_start_ticks = -1;
                motors->direction = 0;
            } else {
                // Move the motors
                gpio_set_level(left_motor->step_pin, 1);
                gpio_set_level(right_motor->step_pin, 1);
                vTaskDelay(1000 / motors->current_speed / 2 / portTICK_PERIOD_MS);
                gpio_set_level(left_motor->step_pin, 0);
                gpio_set_level(right_motor->step_pin, 0);
                vTaskDelay(1000 / motors->current_speed / 2 / portTICK_PERIOD_MS);
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

/**
 * Main function. Runs once on boot.
*/
void app_main(void)
{
    init_app_state();

    config_motor_pins(BO_LEFT_STEP_PIN, BO_LEFT_DIR_PIN);
    config_motor_pins(BO_RIGHT_STEP_PIN, BO_RIGHT_DIR_PIN);
    config_motor_pins(F_LEFT_STEP_PIN, F_LEFT_DIR_PIN);
    config_motor_pins(F_RIGHT_STEP_PIN, F_RIGHT_DIR_PIN);

    // Configure motors
    Motor* blackout_left = malloc(sizeof(Motor));
    Motor* blackout_right = malloc(sizeof(Motor));
    MotorPair* blackout_pair = malloc(sizeof(MotorPair));
    Motor* filter_left = malloc(sizeof(Motor));
    Motor* filter_right = malloc(sizeof(Motor));
    MotorPair* filter_pair = malloc(sizeof(MotorPair));
    // Check for memory allocation errors
    if (
        blackout_left == NULL || blackout_right == NULL || blackout_pair == NULL ||
        filter_left == NULL || filter_right == NULL || filter_pair == NULL
        ) {
        esp_restart();
        printf("Memory allocation error.\n");
        return;
    }
    blackout_left->mutex = xSemaphoreCreateMutex();
    blackout_left->dir_pin = BO_LEFT_DIR_PIN;
    blackout_left->step_pin = BO_LEFT_STEP_PIN;
    blackout_right->mutex = xSemaphoreCreateMutex();
    blackout_right->dir_pin = BO_RIGHT_DIR_PIN;
    blackout_right->step_pin = BO_RIGHT_STEP_PIN;
    blackout_pair->left_motor = blackout_left;
    blackout_pair->right_motor = blackout_right;
    blackout_pair->current_pos = 0;
    blackout_pair->target_pos = 0;
    blackout_pair->current_speed = 0;
    blackout_pair->direction = 0;

    filter_left->mutex = xSemaphoreCreateMutex();
    filter_left->dir_pin = F_LEFT_DIR_PIN;
    filter_left->step_pin = F_LEFT_STEP_PIN;
    filter_right->mutex = xSemaphoreCreateMutex();
    filter_right->dir_pin = F_RIGHT_DIR_PIN;
    filter_right->step_pin = F_RIGHT_STEP_PIN;
    filter_pair->left_motor = filter_left;
    filter_pair->right_motor = filter_right;
    filter_pair->current_pos = 0;
    filter_pair->target_pos = 0;
    filter_pair->current_speed = 0;
    filter_pair->direction = 0;

    if(xSemaphoreTake(app_state.mutex, portMAX_DELAY) == pdTRUE) {
        app_state.state = STATE_IDLE;
    }

    xTaskCreate(run_motors, "blackout_run_task", MOTOR_TASK_STACK_SIZE, (void*)blackout_pair, MOTOR_TASK_PRIORITY, NULL);
    //xTaskCreate(run_motors, "filter_run_task", MOTOR_TASK_STACK_SIZE, (void*)filter_pair, MOTOR_TASK_PRIORITY, NULL);

    wait(3);

    printf("Running to 100...\n");
    blackout_pair->target_pos = 200;
    wait(50);

}

