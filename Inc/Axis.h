/*
 * Axis.h
 *
 *  Created on: Jul 3, 2025
 *      Author: DELL
 *      Currently updating Homing exti
 */
#ifndef INC_AXIS_H_
#define INC_AXIS_H_

#ifdef STM32F103xB
#include "stm32f1xx_hal.h"
#elif defined(STM32F401xC) || defined(STM32F411xE)
#include "stm32f4xx_hal.h"
#endif

#include <stdint.h>

// Direction of axis
typedef enum
{
    CW, // Clockwise
    CCW // Counter-clockwise
} Direction;

// Axis states
typedef enum
{
    Idle,
    Running_Accel,
    Running_Const,
    Homing,
    Error
} Axis_State;

// Main Axis structure
typedef struct
{
    // Configuration for direction control pin
    GPIO_TypeDef *GPIOx_Dir; ///< GPIO port used for direction control
    uint16_t GPIO_Pin_Dir;   ///< GPIO pin number used for direction control

    // Timer handles and configuration
    TIM_HandleTypeDef *htim_Pul;   ///< Timer handle for pulse (PWM) generation
    uint32_t Channel_Pul;          ///< Timer channel used for pulse output
    TIM_HandleTypeDef *htim_Accel; ///< Timer handle for base timer used in acceleration profile

    // Optional limit switches
    GPIO_TypeDef *GPIOx_Limit;
    uint16_t GPIO_Pin_Limit;
    uint8_t invert_home_direction; // Flag to indicate if home direction is inverted
    uint32_t last_limit_triggered; ///< Timestamp of the last limit switch trigger (for debouncing)

    // Timer settings for frequency generation
    uint32_t TIMdivARR;     ///< Timer divider used to calculate prescaler (ARR value)
    uint32_t PSC_Max;       ///< Maximum prescaler value allowed
    uint32_t frequency_min; ///< Minimum movement frequency (start speed)
    uint32_t frequency_max; ///< Maximum movement frequency (max speed)
    uint32_t pulse_per_mm;  ///< Number of pulses needed to move 1mm

    // Motion configuration
    uint8_t invert_dir; ///< Flag to indicate direction inversion
    uint16_t vMax;      ///< Not used currently: Max velocity (for future use)
    uint16_t vMin;      ///< Not used currently: Min velocity (for future use)
    uint16_t aMax;      ///< Not used currently: Max acceleration (for future use)
    uint16_t aMin;      ///< Not used currently: Min acceleration (for future use)

    // Current axis state and direction
    Axis_State state;    ///< Current state of the axis (Idle, Running, etc.)
    Direction direction; ///< Current direction of movement

    // Pulse tracking
    uint32_t pulse_elapsed; ///< Number of pulses that have been completed
    uint32_t pulse_current; ///< Could represent real-time pulse count
    uint32_t pulse_target;  ///< Total number of pulses required for a move

    // Position tracking
    int64_t pos_current; ///< Current position in steps or mm
    int64_t pos_target;  ///< Target position in steps or mm

    // Prescaler and frequency tracking
    uint32_t PSC_Current;           ///< Current timer prescaler value
    uint32_t frequency_current;     ///< Current frequency being applied to the pulse timer
    uint32_t pulse_Max_accel_start; ///< Pulse count when maximum speed is reached (used in deceleration)
    uint32_t pulse_Halfway;         ///< Halfway point in total pulses (used in accel/decel)
    uint16_t frequency_step;        ///< Step size to increase/decrease frequency
} Axis;

// Function declarations
void Axis_Init(Axis *axis,
               GPIO_TypeDef *GPIOx_Dir,
               uint16_t GPIO_Pin_Dir,
               TIM_HandleTypeDef *htim_Pul,
               uint32_t Channel_Pul,
               TIM_HandleTypeDef *htim_Accel,
               uint32_t TIMdivARR,
               uint32_t PSC_Max,
               uint32_t frequency_min,
               uint32_t frequency_max,
               uint32_t pulse_per_mm,
               uint8_t invert_dir

);
void Axis_LimitInit(Axis *axis,
                    GPIO_TypeDef *GPIOx_Limit,
                    uint16_t GPIO_Pin_Limit,
                    uint8_t invert_home_direction);

/**
 * @brief Calculate number of pulses based on current and target position
 */
void Axis_CalculatePulse(Axis *axis);

/**
 * @brief Calculate and set timer prescaler based on desired frequency
 */
void Axis_CalculateAndChangeFrequency(Axis *axis, float frequency);

/**
 * @brief Move to relative position with constant speed
 */
void Axis_RunPosConst(Axis *axis, int64_t pos_relative);

/**
 * @brief Move to absolute target position with constant speed
 */
void Axis_RunToPosConst(Axis *axis, int64_t pos_target);

/**
 * @brief Start axis movement with constant speed (starts PWM)
 */
void Axis_StartAxisConst(Axis *axis);

/**
 * @brief Move to relative position with acceleration
 */
void Axis_RunPosAccel(Axis *axis, int64_t pos_relative, uint16_t freq_step, uint32_t f_min, uint32_t f_max);

/**
 * @brief Move to absolute position with acceleration
 */
void Axis_RunToPosAccel(Axis *axis, int64_t pos_target, uint16_t freq_step, uint32_t f_min, uint32_t f_max);

/**
 * @brief Start axis movement with acceleration profile
 */
void Axis_StartAxisAccel(Axis *axis, uint16_t freq_step, uint32_t f_min, uint32_t f_max);

/**
 * @brief Stop axis movement and reset GPIO direction pin
 */
void Axis_Stop(Axis *axis);

/**
 * @brief Timer PWM pulse callback for constant speed mode
 */
uint8_t Axis_TIM_PWM_PulseFinishedCallback_Const(Axis *axis);

/**
 * @brief Timer PWM pulse callback for acceleration mode
 */
uint8_t Axis_TIM_PWM_PulseFinishedCallback_Accel(Axis *axis);

/**
 * @brief Base timer callback to change frequency for acceleration
 */
uint8_t Axis_TIM_PeriodElapsedCallback_ChangeFreq(Axis *axis);

/**
 * @brief Start homing process
 */
void Axis_StartHoming(Axis *axis);

/*
 * @brief Callback for touch limit switch interrupt
 */
void Axis_GPIO_EXTI_Callback_Limit(Axis *axis, uint16_t GPIO_Pin);

/*
* @brief Handle axis error state
 * This function should be called when an error occurs, such as hitting a limit switch
 */
*/
void Axis_ErrorHandler(Axis *axis);

#endif /* INC_AXIS_H_ */
