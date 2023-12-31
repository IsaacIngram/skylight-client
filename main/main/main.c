#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"

// Pins
#define PIN_ENABLE_ALL 8
#define PIN_DIR_1 9
#define PIN_STEP_1 10
#define PIN_DIR_2 11
#define PIN_STEP_2 12
#define PIN_DIR_3 13
#define PIN_STEP_3 14
#define PIN_DIR_4 15
#define PIN_STEP_4 16
#define PIN_BUTTON 17
#define PIN_LED 18

// Motor characteristics
#define MOTOR_DIRECTION 1
#define MAX_DIST 1000 // steps
#define MAX_SPEED 1000 // steps/s
#define MAX_ACCEL 250 // steps/s

// Task characteristics
#define MOTOR_TASK_STACK_SIZE 2048
#define MOTOR_TASK_PRIORITY (configMAX_PRIORITIES - 1)

typedef struct {

    int target_pos;
    int current_pos;
    int current_speed;

    int dir_pin;
    int step_pin;

    SemaphoreHandle_t mutex;

} Motor;


typedef struct {

    Motor* left_motor;
    Motor* right_motor;

} MotorPair;

static void wait(double seconds) {
    vTaskDelay((seconds * 1000) / portTICK_PERIOD_MS);
}

/**
 * Set speed of a motor
 * @param motor Pointer to the motor to set
 * @param speed Integer speed
*/
static void set_speed(Motor *motor, int speed) {
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    }
    motor->current_speed = speed;
}

/**
 * Run the motor. This function will run continuously and should be scheduled
 * with a xTaskCreate function call.
*/
static void run_motors(void* pvParameters) {

    MotorPair* motors = (MotorPair*)pvParameters;

    Motor* left_motor = motors->left_motor;
    Motor* right_motor = motors->right_motor;

    bool has_lm_mutex = false;
    bool has_rm_mutex = false;


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

        // Check if motors are equalized and at target positions
        if ((left_motor->current_pos == left_motor->target_pos) /*&& (right_motor->current_pos == right_motor->target_pos) && (left_motor->current_pos == right_motor->current_pos)*/)
        {
            // Equalized at target position, stop all motors
            set_speed(left_motor, 0);
            set_speed(right_motor, 0);
            printf("Motors stopped\n");
        } else {
            // Motors need to move

            int direction = 1;

            // Calculate steps required
            int left_steps_to_move = left_motor->target_pos - left_motor->current_pos;
            
            
            // Set direction
            if (left_steps_to_move < 0) {
                direction = -1;
                left_steps_to_move *= -1;
            }
            gpio_set_level(left_motor->dir_pin, direction * MOTOR_DIRECTION);


            // Adjust speed based on acceleration
            if(left_motor->current_speed < MAX_SPEED) {
                set_speed(left_motor, left_motor->current_speed + MAX_ACCEL);
            }

            // Move the motor
            gpio_set_level(left_motor->step_pin, 1);
            printf("Running %d\n", left_motor->current_pos);
            vTaskDelay(1000 / left_motor->current_speed / 2 / portTICK_PERIOD_MS);
            gpio_set_level(left_motor->step_pin, 0);

            left_motor->current_pos += direction;

        }

        
        // Give up mutex locks
        xSemaphoreGive(left_motor->mutex);
        xSemaphoreGive(right_motor->mutex);
        has_lm_mutex = false;
        has_rm_mutex = false;

        // Delay 10ms
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

int step_target = 0;

void app_main(void)
{

    Motor* blackout_left = malloc(sizeof(Motor));
    Motor* blackout_right = malloc(sizeof(Motor));
    MotorPair* blackout_pair = malloc(sizeof(MotorPair));

    // Check for memory allocation errors
    if (blackout_left == NULL || blackout_right == NULL || blackout_pair == NULL) {
        esp_restart();
        printf("Memory allocation error.\n");
        return;
    }

    blackout_pair->left_motor = blackout_left;
    blackout_pair->right_motor = blackout_right;

    blackout_left->mutex = xSemaphoreCreateMutex();
    blackout_left->target_pos = 0;
    blackout_left->current_pos = 0;
    blackout_left->current_speed = 0;
    blackout_left->dir_pin = PIN_DIR_1;
    blackout_left->step_pin = PIN_STEP_1;

    blackout_right->mutex = xSemaphoreCreateMutex();
    blackout_right->target_pos = 0;
    blackout_right->current_pos = 0;
    blackout_right->current_speed = 0;
    blackout_right->dir_pin = PIN_DIR_2;
    blackout_right->step_pin = PIN_STEP_2;

    xTaskCreate(run_motors, "blackout_run_task", MOTOR_TASK_STACK_SIZE, (void*)blackout_pair, MOTOR_TASK_PRIORITY, NULL);

    wait(3);

    printf("STARTING FORWARD MOTION\n");

    blackout_left->target_pos = 1000;
    wait(10);

}

