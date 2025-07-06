# 🚀 STM32 Axis Motion Library

A clean, extensible, and powerful **C library** for controlling **stepper motors** using STM32 and HAL drivers.  
Supports acceleration, constant speed, limit switches, and homing operations.

---

## 🔧 Features

- ✅ Direction control (CW/CCW)  
- ✅ Acceleration & constant speed motion  
- ✅ Timer-based PWM generation  
- ✅ Limit switch support (with debounce)  
- ✅ Homing function  
- ✅ Easy integration with STM32 HAL  
- ✅ State machine management  

---

## 📁 File Structure

| File     | Description                          |
|----------|--------------------------------------|
| Axis.h   | Struct definitions and function APIs |
| Axis.c   | Core implementation                  |

---

## 🛠️ Getting Started

### 1️⃣ Add Files

Copy `Axis.h` and `Axis.c` into your STM32 project.  
Include the files in your IDE/project settings.

---

### 2️⃣ Hardware Setup

| Pin Type  | Purpose                    |
|-----------|----------------------------|
| DIR       | Direction control (GPIO)   |
| STEP/PWM  | Pulse generation (Timer)   |
| LIMIT     | Limit switch (optional)    |

Configure these using STM32CubeMX or HAL.

---

### 3️⃣ Initialization Example

```c
Axis axisX;

Axis_Init(&axisX,
          DIR_GPIO_Port, DIR_Pin,
          &htimX, TIM_CHANNEL_X,
          &htimAccel,
          TIMdivARR,
          PSC_Max,
          frequency_min,
          frequency_max,
          pulse_per_mm,
          invert_dir);

Axis_LimitInit(&axisX, LIMIT_GPIO_Port, LIMIT_Pin, invert_home_direction);
```
## 🚀 Basic Usage

### ▶️ Constant Speed

```c
Axis_RunPosConst(&axisX, +100);    // Move +100 mm (relative)
Axis_RunToPosConst(&axisX, 0);     // Move to absolute position 0 
```

## ⚡ Accelerated Motion
```c
Axis_RunPosAccel(&axisX, 50, 5, 100, 1000); // Move +50 mm with acceleration
```
## 🏠 Homing
```c
Axis_StartHoming(&axisX); // Start homing process (limit switch required)
```
## 🔁 Timer Integration
Add this to your STM32 timer callback functions:
```c
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    Axis_TIM_PWM_PulseFinishedCallback_Const(&axisX); // Or _Accel
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    Axis_TIM_PeriodElapsedCallback_ChangeFreq(&axisX);
}
```
## ⛔ Limit Switch Interrupt
In your EXTI interrupt handler:
```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    Axis_GPIO_EXTI_Callback_Limit(&axisX, GPIO_Pin);
}
```
## 📊 Axis States
| State           | Description                   |
| --------------- | ----------------------------- |
| `Idle`          | Motor is stopped              |
| `Running_Const` | Moving at constant speed      |
| `Running_Accel` | Accelerating or decelerating  |
| `Homing`        | Homing in progress            |
| `Error`         | Error state (e.g., limit hit) |


## ⚙️ Configuration Tips
- TIMdivARR = Timer_Clock / Desired_Frequency

- pulse_per_mm = steps_per_rev / lead_screw_pitch × microstepping

Use different timers for:

- Pulse generation (PWM mode)
- Frequency ramping (base timer in interrupt mode)

## 📈 Acceleration Profile
```
            Acceleration Profile

Speed
  ^
  |              /‾‾‾‾‾‾‾‾‾\
  |             /           \
  |            /             \
  |___________/               \__________
        Start          Mid       End
