# Energy-based swing-up and Stateflow switching

The [current hybrid model](../models/furuta_hybrid_model.slx) connects the supervisor and controller to **System**, which contains the switching **H-Bridge**, **DC Motor**, and **Pendulum Plant**. Open **Swing-Up** to inspect **Pendulum Energy**, **Arm Acceleration Command**, and **Torque and Motor Voltage**. Stateflow selects which command reaches the motor. Run `verify_furuta_swingup_blocks` to compare the block diagram against the existing swing-up function. Its gains and swing-up settings are provisional. The instructions and results below describe the earlier example-parameter model. For the current model's interfaces and checks, see [CAD plant and controller wiring](cad-controller-wiring.md).

Run from the project folder in MATLAB:

```matlab
build_furuta_hybrid
verify_furuta_hybrid
open_system(fullfile('models','furuta_hybrid_example.slx'))
```

Press Run. The pendulum starts at theta1 = pi, hanging at rest. The Stateflow Supervisor selects Swing-Up, Stabilization or Safe-Stop. Open Hybrid Controller to inspect the MATLAB Function that selects the voltage command. Choose the stabilization law before pressing Run. Double-click Controller Selection and set its Value:
    
| Value | Stabilization controller |
| --- | --- |
| 1 | PID/PD |
| 2 | Pole placement |
| 3 | LQR, the default |

The same Stateflow chart handles all three choices. The chosen controller only commands the motor during Stabilization. The CSV output includes ControllerId so each experiment identifies the active selection.

## Swing-up law

The pendulum energy is E = 0.5*Jp*omega1^2 + m1*g*l1*cos(theta1), with Jp = J1cm + m1*l1^2. The upright energy is m1*g*l1. The controller creates a desired arm acceleration:

```text
a0 = 100*(E - E_upright - 0.04)*tanh(8*omega1*cos(theta1))
     - 4*theta0 - 3*omega0
```

The small energy offset is an empirical pumping margin for the example plant with friction and arm centering. Arm position and speed terms restrain travel. A 3 rad/s^2 starting push is added at near-zero pendulum rate when E is negative. Desired acceleration is limited to +/-25 rad/s^2.

The second plant equation supplies the implied pendulum acceleration. The first equation then supplies the required torque, which is converted to DC motor voltage using the existing motor model. This mapping includes gravity, velocity coupling, viscous friction and back EMF. The result is limited to +/-12 V.

All gains, guard thresholds and timing are in [furuta_swingup_settings.m](../src/controllers/furuta_swingup_settings.m). The model embeds a snapshot of the settings and controller functions at build time.

## Switching and timing

Sensors are sampled and the supervisor/controller update every 0.005 seconds. The continuous plant is integrated with a maximum solver step of 0.001 seconds. Pendulum angle is wrapped to [-pi,pi] for switching and stabilization feedback.

| State | Command and transitions |
| --- | --- |
| Swing-Up | Apply the energy controller. Enter Stabilization when upright error is below 18 deg, pendulum speed below 3 rad/s, arm speed below 3 rad/s, and predicted initial LQR voltage below 10 V. PID/PD and pole placement also require their own predicted initial voltage below 10 V. The shared LQR check retains the tested capture region. |
| Stabilization | Apply the selected stabilization controller. Return to Swing-Up if upright error exceeds 35 deg or pendulum speed exceeds 6 rad/s. |
| Safe-Stop | Command zero voltage and latch until a simulation restart. An invalid controller selection, disable signal, nonfinite state, arm travel reaching +/-180 deg or either angular speed reaching 20 rad/s requests this state. |

Stop transitions have priority over capture and fallback. Separate capture and release thresholds give hysteresis. At capture the supervisor records the current arm angle as the position reference. The selected controller then regulates that arm reference and zero pendulum angle. The final arm angle can therefore differ from its initial zero.

The model workspace variables `enableTime` and `disableTime` control the demonstration enable signal. Their defaults enable at zero and disable beyond the simulation horizon. Mode numbers are 1 for Swing-Up, 2 for Stabilization and 0 for Safe-Stop.

The state-machine implementation follows the [MathWorks Stateflow programmatic interface](https://www.mathworks.com/help/stateflow/programmatic-interface.html).

## Validation and hardware work

Verification runs each of the three controller selections from hanging at pi and pi +/- 0.1 rad, checks capture and sustained balance, and records CSV traces and a plot under `output/hybrid/`. A separate harness exercises the actual Stateflow chart with prescribed inputs to check capture, fallback, stop priority and the stop latch.

These are example-parameter simulation settings. The +/-180 deg travel limit, 20 rad/s rate limit and 12 V voltage limit require replacement with hardware limits. Zero simulated motor voltage includes electrical damping in the plant model. Physical motor-driver disable and emergency-stop wiring still need implementation and testing.

Measured parameters, encoder calibration and rate estimates are required for hardware integration. The bent Allen-key geometry and its inertia axes still need to be reconciled with the idealized plant model. Use the measured build and verification options once the parameter table is complete.

## Verified example results

| Initial hanging offset | First LQR capture | Peak arm travel from zero | Peak voltage |
| --- | --- | --- | --- |
| 0 rad | 4.865 s | 159.781 deg | 12 V |
| -0.1 rad | 5.025 s | 154.302 deg | 12 V |
| +0.1 rad | 5.035 s | 153.642 deg | 12 V |

All three runs remained in Stabilization after first capture and finished within 0.1 degree of upright and the captured arm reference. The Stateflow harness also passed fallback, disable, travel-limit priority, speed-limit and stop-latch checks. These earlier LQR results are retained as a reference. The updated verification table includes all three selections.

The jumps between -180 and +180 degrees in the pendulum plot come from angle wrapping at the hanging direction.

    
## PID state handling

The hybrid controller is sampled at 5 ms. On entering PID stabilization, the integral voltage starts at zero and the low-pass states are initialized from the current angles and angular rates. The filter states then use exact first-order low-pass updates and the integral uses a forward-Euler update with saturation back-calculation. This is the sampled implementation of the existing PID/PD law.

PID memory is reset while swing-up or safe stop is active. Selecting PID during stabilization also initializes its states. Voltage limiting remains active for every choice.

The controller files and gain snapshots are embedded when the model is built. Rebuild after changing gains, parameters or settings. The current simulations use exact plant angular rates, which need estimates or an observer when integrating encoders.

Use the Controller column in output/hybrid/hybrid_metrics_example.csv to compare capture time, peak arm travel, peak voltage and final errors for all nine hanging-start experiments. Each controller also has a nominal response plot in the same folder.


## Selector validation results

All nine hanging-start cases passed with the selectable controller model. Each remained in Stabilization after capture. The chart checks also passed for all three choices and for an invalid selection. The nominal capture time is 4.865 s for each choice because the swing-up law and shared capture region are the same.

| Controller | Time after capture to stay within recovery bands | Post-capture RMS voltage |
| --- | --- | --- |
| PID_PD | 4.563 s | 0.276 V |
| PolePlacement | 2.176 s | 0.073 V |
| LQR | 0.742 s | 0.201 V |

Recovery bands are 0.5 deg for pendulum angle and 2 deg for arm error relative to the capture reference. These measurements describe the example model and the finite simulation interval.
