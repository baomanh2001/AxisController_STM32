/*
 * Axis.c
 *
 *  Created on: Jul 3, 2025
 *      Author: DELL
 */

#include "Axis.h"

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
               uint8_t invert_dir) {
    axis->GPIOx_Dir = GPIOx_Dir;
    axis->GPIO_Pin_Dir = GPIO_Pin_Dir;
    axis->htim_Pul = htim_Pul;
    axis->Channel_Pul = Channel_Pul;
    axis->htim_Accel = htim_Accel;
    axis->TIMdivARR = TIMdivARR;
    axis->PSC_Max = PSC_Max;
    axis->frequency_min = frequency_min;
    axis->frequency_max = frequency_max;
    axis->pulse_per_mm = pulse_per_mm;
    axis->invert_dir = invert_dir;
    axis->last_limit_triggered = 0; // Initialize last limit triggered timestamp
    axis->state = Idle;
}

void Axis_LimitInit(Axis *axis,
                    GPIO_TypeDef *GPIOx_Limit,
                    uint16_t GPIO_Pin_Limit,
                    uint8_t invert_home_direction) {
    axis->GPIOx_Limit = GPIOx_Limit;
    axis->GPIO_Pin_Limit = GPIO_Pin_Limit;
    axis->invert_home_direction = invert_home_direction;
}

/*
Calculate the number of pulses needed to move from pos_current to pos_target
*/
void Axis_CalculatePulse(Axis *axis) {
    int64_t diff = axis->pos_target - axis->pos_current;
    if (diff < 0) {
        axis->direction = axis->invert_dir ? CW : CCW;
        diff = -diff;
    }
    else {
        axis->direction = axis->invert_dir ? CCW : CW;
    }
    axis->pulse_target = diff * axis->pulse_per_mm;
}

/*
Calculate and set timer prescaler based on desired frequency
*/
void Axis_CalculateAndChangeFrequency(Axis *axis, float frequency) {
    uint64_t PSC_new = axis->TIMdivARR / frequency * 1.01348;
    if (PSC_new > axis->PSC_Max) {
        PSC_new = axis->PSC_Max;
    }
    axis->PSC_Current = PSC_new;
    axis->htim_Pul->Instance->PSC = axis->PSC_Current;
}

/*
Set relative position with constant speed
*/
void Axis_RunPosConst(Axis *axis, int64_t pos_relative) {
    axis->pos_target = axis->pos_current + pos_relative;
    Axis_RunToPosConst(axis, axis->pos_target);
}

/*
Set absolute target position with constant speed
*/
void Axis_RunToPosConst(Axis *axis, int64_t pos_target) {
    axis->pos_target = pos_target;
    Axis_CalculatePulse(axis);
    if(axis->pulse_target !=0)
    {
    axis->pulse_elapsed = 0;
    Axis_StartAxisConst(axis);
    }
    else
    {
        axis->state = Idle;
    }
}

/*
Start axis movement with constant speed (starts PWM)
*/
void Axis_StartAxisConst(Axis *axis) {
    if (axis->state == Idle || axis->state == Error) {
        axis->state = Running_Const;
        HAL_GPIO_WritePin(axis->GPIOx_Dir, axis->GPIO_Pin_Dir, (axis->direction == CW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_TIM_PWM_Start_IT(axis->htim_Pul,axis->Channel_Pul);
    }
}

/*
Stop axis movement and reset GPIO direction pin
*/
void Axis_Stop(Axis *axis) {
    if (axis->state == Running_Const|| axis->state == Running_Accel || axis->state == Error) {
        axis->state = Idle;
        HAL_TIM_PWM_Stop_IT(axis->htim_Pul,axis->Channel_Pul);
        HAL_GPIO_WritePin(axis->GPIOx_Dir, axis->GPIO_Pin_Dir, GPIO_PIN_RESET);
    }
}

/*
Set relative position with acceleration
*/
void Axis_RunPosAccel(Axis *axis, int64_t pos_relative, uint16_t freq_step, uint32_t f_min, uint32_t f_max) {
    axis->pos_target = axis->pos_current + pos_relative;
    Axis_RunToPosAccel(axis, axis->pos_target, freq_step, f_min, f_max);
}

/*
Set absolute position with acceleration
*/
void Axis_RunToPosAccel(Axis *axis, int64_t pos_target, uint16_t freq_step, uint32_t f_min, uint32_t f_max) {
    axis->pos_target = pos_target;
    Axis_CalculatePulse(axis);
    if(axis->pulse_target != 0)
    {
    axis->pulse_elapsed = 0;
    axis->pulse_Halfway = axis->pulse_target / 2;
    axis->pulse_Max_accel_start = 0;
    Axis_StartAxisAccel(axis, freq_step, f_min, f_max);
    }
    else
    {
        axis->state = Idle;
    }
}

/*
Start axis movement with acceleration profile (starts PWM and TIM for acceleration)
*/
void Axis_StartAxisAccel(Axis *axis, uint16_t freq_step, uint32_t f_min, uint32_t f_max) {
    if (axis->state == Idle || axis->state == Error) {
        HAL_GPIO_WritePin(axis->GPIOx_Dir, axis->GPIO_Pin_Dir, (axis->direction == CW) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        axis->state = Running_Accel;
        HAL_TIM_Base_Start_IT(axis->htim_Accel);
        axis->frequency_min = f_min;
        axis->frequency_max = f_max;
        axis->frequency_step = freq_step;
        axis->frequency_current = f_min;
        Axis_CalculateAndChangeFrequency(axis, axis->frequency_current);
        HAL_TIM_PWM_Start_IT(axis->htim_Pul, axis->Channel_Pul);
    }
}

/*
Callback for pulse generation at constant speed.
*/
uint8_t Axis_TIM_PWM_PulseFinishedCallback_Const(Axis *axis) {
    axis->pulse_elapsed++;
    if (axis->pulse_elapsed >= axis->pulse_target) {
    	axis->pos_current = axis->pos_target;
        Axis_Stop(axis);
        return 1;
    }
    return 0;
}

/*
Callback for pulse generation at acceleration.
*/
uint8_t Axis_TIM_PWM_PulseFinishedCallback_Accel(Axis *axis) {

    axis->pulse_elapsed++;
    if (axis->pulse_elapsed >= axis->pulse_target) {
        Axis_Stop(axis);
        axis->pos_current = axis->pos_target;
        return 1;
    }
    return 0;
}

/*
Callback for frequency change
*/
uint8_t Axis_TIM_PeriodElapsedCallback_ChangeFreq(Axis *axis) {
    // Already reached Maximum acceleration, so need to determine if need to decelerate
    if (axis->pulse_Max_accel_start != 0) {
        // Check if the pulse elapsed is more than the halfway point + Maximum acceleration start
        /*
        Speed graph for acceleration
                                _________________________________________
                                |           Max Half  Max               |
                                |         Speed  way   Speed            |
                                |            |   |    |                 |
                                |                |                      |
                                |            /‾‾‾|‾‾‾‾\                 |
                                |           /    |     \                |
                                |          /     |      \               |
                                |         /      |       \              |
                                |        /_______|________\             |
                                |        |       |        |             |
                                |        0     Half       Target        |
                                |        0      way       Pulse         |
                                |_______________________________________|
                                            SPEED GRAPH

        */
        if (axis->pulse_elapsed >= -axis->pulse_Max_accel_start + axis->pulse_Halfway+ axis->pulse_Halfway) {
            axis->frequency_current -= axis->frequency_step; // Deccelerate by reducing the frequency
            if (axis->frequency_current < axis->frequency_min)
                axis->frequency_current = axis->frequency_min;// Ensure frequency does not go below minimum
            Axis_CalculateAndChangeFrequency(axis, axis->frequency_current);//update frequency
            return 2;                   // Return 2 to indicate the Decceleration
        }
        return 1; // Return 1 to indicate the frequency change is max speed
    } 
    else // if axis->pulse_Max_accel_start == 0) // If not yet reached maximum acceleration
    {
        // Check if the pulse elapsed is more than the halfway point
        /* Speed graph for acceleration
                                        _______________________
                                        |      /|\            |
                                        |     / | \           |
                                        |    /  |  \          |
                                        |   /   |   \         |
                                        |  /____|____\        |
                                        |  |    |     |       |
                                        |  0   Half   Target  |
                                        |  0    way    Pulse  |
                                        |_____________________|
                                            SPEED GRAPH

        */
        if (axis->pulse_elapsed >= axis->pulse_Halfway) 
        {
            axis->frequency_current -= axis->frequency_step; // Deccelerate by reducing the frequency
            if (axis->frequency_current < axis->frequency_min)// Ensure frequency does not go below minimum
                axis->frequency_current = axis->frequency_min;
            Axis_CalculateAndChangeFrequency(axis, axis->frequency_current);//update frequency
            return 2;// Return 2 to indicate the Decceleration
        } else {
            // If not yet reached halfway, continue accelerating
            // Check if the frequency has reached maximum
            if (axis->frequency_current + axis->frequency_step >= axis->frequency_max) {
                axis->frequency_current = axis->frequency_max;// Set to maximum frequency
                axis->pulse_Max_accel_start = axis->pulse_elapsed;// Set the pulse count when maximum speed is reached
            } else {
                // If not yet reached maximum frequency, increase the frequency
                axis->frequency_current += axis->frequency_step;
            }
            Axis_CalculateAndChangeFrequency(axis, axis->frequency_current);//update frequency
        }
    }
    return 0;// Return 0 to indicate the frequency change is still accelerating
}

/*
Start homing process
*/
void Axis_StartHoming(Axis *axis) {
    if (axis->state == Idle || axis->state == Error) {
        axis->state = Homing;
        HAL_GPIO_WritePin(axis->GPIOx_Dir, axis->GPIO_Pin_Dir, (axis->invert_home_direction) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        HAL_TIM_PWM_Start(axis->htim_Pul, axis->Channel_Pul);
    }
}

/*
Callbacck for touch limit switch interrupt
*/
void Axis_GPIO_EXTI_Callback_Limit(Axis *axis, uint16_t GPIO_Pin) 
{
    if (GPIO_Pin != axis->GPIO_Pin_Limit)
        return; // Check if the correct pin triggered the interrupt
    // If limit switch is triggered, stop the axis
    // Check if the limit switch is already triggered 
    // If it is triggered within 50ms, ignore the trigger
    // CAN CHANGE THIS TIME BASED ON YOUR REQUIREMENTS (50ms is a good debounce time)
    else if(HAL_GetTick() - axis->last_limit_triggered > 50) // Debounce limit switch trigger
    {
        axis->last_limit_triggered = HAL_GetTick(); // Update last limit triggered timestamp
        Axis_LimitSwitchTriggered(axis);
        if (axis->state == Running_Accel || axis->state == Running_Const)
        {
            Axis_Stop(axis);
            axis->state = Error; // Set state to Error if limit switch is triggered
        }
        else if (axis->state == Homing)
        {
            // If homing is in progress, stop and set position to zero
            Axis_Stop(axis);
            axis->pos_current = 0; // Reset current position to zero
            axis->state = Idle;    // Set state back to Idle after homing
        }
    }
}

void Axis_ErrorHandler(Axis *axis)
 {
    /*
    Add your error handling logic here.
    This could include logging the error, resetting the axis state, etc.
    */
}
