# Simulink Controller Architecture & Details

# Input Variables (Unity -> Simulink):
1. Theta
2. Phi
3. Psi
4. Unity_X_FB
5. Unity_Y_FB
6. Barometer_ALT (ALT_FB input of Altitude Controller & Altitude_FB of Stateflow Chart)
7. LiDAR_ALT
8. Unity_X_SP
9. Unity_Y_SP
10. Unity_ALT_SP
11. Unity_Yaw_SP

# Local Model Variables (Set by "Constant" blocks):
1. Command
2. Landing_Speed
3. GPS_Error_R
4. Low_Volt_Flag

# Frame Conversion MATLAB Function:
Transforms the position error from the world frame to the local drone frame.
```
function [Err_Body_X, Err_Body_Y] = transform_error_to_body(PSI_deg, POS_X_SP, POS_X_FB, POS_Y_SP, POS_Y_FB)
    error_global_x = POS_X_SP - POS_X_FB;
    error_global_y = POS_Y_SP - POS_Y_FB;
    
    Err_Body_X =  (error_global_x * cosd(PSI_deg)) + (error_global_y * sind(PSI_deg));
    Err_Body_Y = -(error_global_x * sind(PSI_deg)) + (error_global_y * cosd(PSI_deg));
end
```

# Stateflow Supervisor Chart (Works in Parallel Execution):
## Main (Parent State of Execution Order: 2):
### States:
#### 1. Standby (Default Transition):
```
en:
Drone_Status = 0;
Controller_Enable = 0;
TakeOff_Complete = 0;
Position_X_SP = 0;
Position_Y_SP = 0;
Mission_Complete = 0;
Drone_Near_Ground = 1;
```
#### 2. On_Ground:
```
en:
Drone_Near_Ground = 1;
Controller_Enable = 0;
Altitude_SP = 0;
Drone_Status = 1;
Position_X_SP = 0;
Position_Y_SP = 0;
```
#### 3. Take_Off:
```
en:
TakeOff_Complete = 0;
Controller_Enable = 1;
Altitude_SP = Unity_ALT_SP;
du:
Drone_Status = 2;
Controller_Enable = 1;
Altitude_SP = Unity_ALT_SP;
if(Altitude_FB < (Altitude_SP + 0.05) && Altitude_FB > (Altitude_SP - 0.05))
    TakeOff_Complete = 1;
else
    TakeOff_Complete = 0;
end
```
#### 4. In_Air:
```
en:
Drone_Status = 3;
Altitude_SP = Unity_ALT_SP;
Controller_Enable = 1;
Position_X_SP = 0;
Position_Y_SP = 0;
Mission_Complete = 0;
du:
Drone_Status = 3;
Altitude_SP = Unity_ALT_SP;
Controller_Enable = 1;
Position_X_SP = Unity_X_SP;
Position_Y_SP = Unity_Y_SP;
if(Position_Error_R <= GPS_Error_R)
    Mission_Complete = 1;
else
    Mission_Complete = 0;
end
```
#### 5. Landing:
```
en:
Controller_Enable = 1;
Drone_Status = 4;
if(Altitude_FB <= 0.2)
    Drone_Near_Ground = 0;
else
    Drone_Near_Ground = 1;
end
du:
Controller_Enable = 1;
Drone_Status = 4;
if(Altitude_FB <= 0.2)
    Drone_Near_Ground = 0;
else
    Drone_Near_Ground = 1;
end
```
#### 6. Stay_On_Ground:
```
en:
Controller_Enable = 0;
Altitude_SP = 0;
Drone_Status = 5;
```
#### 7. Crash:
```
en:
Controller_Enable = 0;
Drone_Status = 6;
du:
Controller_Enable = 0;
Drone_Status = 6;
```
### Transitions:
#### Standby -> On_Ground: 
```[after(1, sec)]```
#### On_Ground -> Take_Off: 
```[Command == 2 && Low_Volt_Flag == 0 && GPS_Ready == 1]```
#### Take_Off -> In_Air (1): 
```[after(5, sec) && TakeOff_Complete == 1]```
#### Take_Off -> Landing (2): 
```[Command == 1 || Low_Volt_Flag == 1]```
#### Take_Off -> Crash (3): 
```[Altitude_FB <= 1 && (abs(Theta) >= 20 || abs(Phi) >= 20)]```
#### In_Air -> Landing (1): 
```[Command == 1 || Low_Volt_Flag == 1 || (after(5, sec) && Mission_Complete == 1)]```
#### In_Air -> Crash (2):
```[Altitude_FB <= 1 && (abs(Theta) >= 20 || abs(Phi) >= 20)]```
#### Landing -> Junction (1): 
```[after(0.01, sec) && Altitude_SP > 0 && Altitude_FB > 0.05]```
#### Junction -> Landing: 
```{Altitude_SP = Altitude_SP - (0.01 * Landing_Speed)}```
#### Landing -> Stay_On_Ground (2): 
```[Altitude_FB <= 0.05]```
#### Landing -> Crash (3): 
```[Altitude_FB <= 1 && (abs(Theta) >= 20 || abs(Phi) >= 20)]```
#### Stay_On_Ground -> On_Ground: 
```[Command == 1]```

## IMU (Parent State of Execution Order: 1):
### States:
#### 1. Initialization (Default Transition):
```
en:
Pitch = 0;
Roll = 0;
Psi = 0;
```
#### 2. Correction:
```
du:
Pitch = Theta;
Roll = Phi;
Yaw = Psi;
```
### Transitions:
#### Initialization -> Correction: 
```
[after(1, sec)]
```
## GPS (Parent State of Execution Order: 3):
### States:
#### 1. Initialization (Default Transition):
```
en:
GPS_Ready = 0;
GNSS_Error_R = GPS_Error_R;
POS_Controller_Enable = 1;
Position_Error_R = 0;
Position_X_FB = 0;
Position_Y_FB = 0;
```
#### 2. Error_Calculation:
```
en:
Position_X_FB = Unity_X_FB;
Position_Y_FB = Unity_Y_FB;
Position_Error_R = (((Position_X_SP - Position_X_FB) ^ 2 + (Position_Y_SP - Position_Y_FB) ^ 2) ^ 0.5);
if(Position_Error_R >= GPS_Error_R)
    POS_Controller_Enable = 1 * Drone_Near_Ground;
else
    POS_Controller_Enable = 0;
end
GPS_Ready = 1;
```
### Transitions:
#### Initialization -> Error_Calculation:
```
[after(1, sec) && Command == 2]
```

# PID MATLAB Functions:
## Variables:
- MAX_Altitude = 8 meters
- Ts = 0.01 sec
- MAX_POS_SP = 5 meters
- MAX_RP_Angle = 20 degrees

## Altitude Controller:
### Notes:
1. The error input of the function (ALT_SP - ALT_FB) * 1 / MAX_Altitude.
2. The output Control_Signal is passed through a Saturation block (0, 1) to a Saturation block (0, Lift_Bound) to a Goto label (ALT_C).

```
function Control_Signal = fcn(Kp,Ki,Kd,Ts, CONT_EN, error)
    persistent error_1 I
    if isempty(error_1)
        error_1=0;
        I=0;
    end
    if(Ki>0)
      I=(I+Ts*(error+error_1)/2)*CONT_EN;
      if(abs(I)>1/Ki)
          I=sign(I)/Ki;
      end
    else
        I=0;
    end
    Control_Signal=((Kp*error+Ki*I+Kd*(error-error_1)/Ts) + 0.404)*CONT_EN;
    % Add a constant hover thrust percentage of 40.4%
    error_1 = error;
end
```

## Position_X_Controller:
### Note:
1. The error input of the function Err_Body_X * 1/MAX_POS_SP.
2. The CONT_EN of the function is (POS_Controller_EN * Controller_Enable).
3. The output Control_Signal is passed through a Saturation block (-1, 1) to a Goto label (POS_X_C).

```
function Control_Signal = fcn(Kp,Ki,Kd,Ts, CONT_EN, error)
    persistent error_1 I
    if isempty(error_1)
        error_1=0;
        I=0;
    end
    if(Ki>0)
      I=(I+Ts*(error+error_1)/2)*CONT_EN;
      if(abs(I)>1/Ki)
          I=sign(I)/Ki;
      end
    else
        I=0;
    end
    Control_Signal=(Kp*error+Ki*I+Kd*(error-error_1)/Ts)*CONT_EN;
    error_1 = error;
end
```

## Position_Y_Controller:
### Note:
1. The error input of the function Err_Body_Y * 1/MAX_POS_SP.
2. The CONT_EN of the function is (POS_Controller_EN * Controller_Enable).
3. The output Control_Signal is passed through a Saturation block (-1, 1) to a Goto label (POS_Y_C).
```
function Control_Signal = fcn(Kp,Ki,Kd,Ts, CONT_EN, error)
    persistent error_1 I
    if isempty(error_1)
        error_1=0;
        I=0;
    end
    if(Ki>0)
      I=(I+Ts*(error+error_1)/2)*CONT_EN;
      if(abs(I)>1/Ki)
          I=sign(I)/Ki;
      end
    else
        I=0;
    end
    Control_Signal=(Kp*error+Ki*I+Kd*(error-error_1)/Ts)*CONT_EN;
    error_1 = error;
end
```

## Pitch_Controller:
### Note:
1. The error input of the function [(POS_Y_C * MAX_RP_Angle) - Pitch_FB] * 1/90.
2. The output Control_Signal is passed through a Saturation block (-RP_Bound, RP_Bound) to a Goto label (PITCH_C). 

```
function Control_Signal = fcn(Kp,Ki,Kd,Ts, CONT_EN, error)
     persistent error_1 I
    if isempty(error_1)
        error_1=0;
        I=0;
    end
    I=(I+Ts*(error+error_1)/2)*CONT_EN;
    Control_Signal=(Kp*error+Ki*I+Kd*(error-error_1)/Ts)*CONT_EN;
    error_1 = error;
end
```

## Roll_Controller:
### Note:
1. The error input of the function [(POS_X_C * MAX_RP_Angle) - Roll_FB] * 1/90.
2. The output Control_Signal is passed through a Saturation block (-RP_Bound, RP_Bound) to a Goto label (ROLL_C). 
```
function Control_Signal = fcn(Kp,Ki,Kd,Ts, CONT_EN, error)
   persistent error_1 I
    if isempty(error_1)
        error_1=0;
        I=0;
    end
    I=(I+Ts*(error+error_1)/2)*CONT_EN;
    Control_Signal=(Kp*error+Ki*I+Kd*(error-error_1)/Ts)*CONT_EN;
    error_1 = error;
end
```

## Yaw_Controller:
### Note:
1. The error input of the function (UNITY_YAW_SP - Yaw_FB) * 1/180.
2. The output Control_Signal is passed through a Saturation block (-Yaw_Bound, Yaw_Bound) to a Goto label (YAW_C). 
```
function Control_Signal = fcn(Kp,Ki,Kd,Ts, CONT_EN, error)
    persistent error_1 I
    if isempty(error_1)
        error_1=0;
        I=0;
    end
    if(Ki>0)
      I=(I+Ts*(error+error_1)/2)*CONT_EN;
      if(abs(I)>1/Ki)
          I=sign(I)/Ki;
      end
    else
        I=0;
    end
    Control_Signal=(Kp*error+Ki*I+Kd*(error-error_1)/Ts)*CONT_EN;
    error_1 = error;
end
```

# Output System:
## Linearization Function:
```
% Thrust_Max=1084 grams
% PWM_Max 255
% Original unnormalized equation is y = 0.0106x^2 + 1.5276x + 5.0422
% where x is PWM and y is the actual Thrust
% Data collected at operating voltage Vdc=12V

function PWM = Linearization(Norm_Thrust)
    PWM_Max = 255;
    PWM=(-0.3592+sqrt(((0.3592)^2)-4*0.6359*...
        (0.00465-Norm_Thrust)))/(2*0.6359);
    if (PWM<0)
        PWM=0;
    end
    PWM=round(PWM*PWM_Max);
    if (PWM>255)
        PWM=255;
    elseif (PWM<0)
        PWM=0;
    end
end
```
## Motor Mixers:
- (ALT_C + PITCH_C + ROLL_C - YAW_C) is passed through a Saturation block (0, 1), to the Norm_Thrust input of the Linearization function, into a Saturation block (0, 255) and to a Goto label (PWM_RL).
- (ALT_C + PITCH_C - ROLL_C + YAW_C) is passed through a Saturation block (0, 1), to the Norm_Thrust input of the Linearization function, into a Saturation block (0, 255) and to a Goto label (PWM_RR).
- (ALT_C - PITCH_C + ROLL_C + YAW_C) is passed through a Saturation block (0, 1), to the Norm_Thrust input of the Linearization function, into a Saturation block (0, 255) and to a Goto label (PWM_FL).
- (ALT_C - PITCH_C - ROLL_C - YAW_C) is passed through a Saturation block (0, 1), to the Norm_Thrust input of the Linearization function, into a Saturation block (0, 255) and to a Goto label (PWM_FR).

Then a From label from every motor signal is passed through a Data Type Conversion block (Output: Single), to a Byte Pack block and to a TCP/IP Send block.

## Output Variables (Simulink -> Unity):
1. PWM_RL
2. PWM_RR
3. PWM_FL
4. PWM_FR