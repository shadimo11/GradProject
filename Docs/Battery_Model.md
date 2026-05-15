# Battery Model Subsystem
## Model Parameters:
### Battery parameters (3S LiPo)
Batt.C_Ah = 5.2              % Capacity [Ah]
Batt.R0   = 0.018            % Ohmic resistance [Ohm]
Batt.Rp   = 0.02             % Polarization resistance [Ohm]
Batt.Cp   = 500              % Polarization capacitance [F]
Max_PWM = 255
Max_Current_per_Motor = 12
Max_Current = Max_Current_Per_Motor * 4

### OCV-SOC lookup table (PACK voltages)
Batt.SOC_vec = [0    0.1    0.2   0.5   0.8   1.0]
Batt.OCV_vec = [9.0  10.95  11.3  11.7  12.1  12.6]  [V]
Batt.SOC_init = 1.0         % Initial SOC
Batt.Vp_init  = 0.0         % Initial polarization voltage
## Inputs:
- PWM input
## Subsystems:
### Input Conversion:
Inport (PWM Array) >> Gain (1 / Max_PWM) >> Math Function block (Power 2) >> Gain (Max_Current_Per_Motor) >> Add Block (+ Idle Current) >> I_Battery Signal
### SoC Dynamics:
I_Battery Signal >> Gain (-1 / Batt.C_Ah * 3600) >> Integrator (Initial Condition = Batt.SoC_init, Saturation limits: 0-1) >> 1-D lookup table (Table data: Batt.OCV_vec, Breakpoints: Batt.SOC_vec) >> OCV_Signal

Note: The signal after the integrator and before the 1-D lookup table is the SoC output.

### Polarization:
I_Battery Signal >> Gain (1 / Batt.Cp) >> Add Block (+)
Vp_Signal >> Gain (-1 / (Batt.Rp * Batt.Cp)) >> Add block (+)
Add block output >> Integrator (Initial condition: Batt.Vp_init) >> Vp_Signal

### Voltage Drop:
I_Battery Signal >> Gain (Batt.R0) >> V_R0_Signal

## Outputs: 
- SoC: The signal after the integrator and before the 1-D lookup table in the SoC Dynamics subsystem.
- V_Battery: Vp_Signal + OCV_Signal + V_R0_Signal