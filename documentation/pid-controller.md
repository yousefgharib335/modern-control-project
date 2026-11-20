# PID upright stabilization

Open the project folder in MATLAB and run:

```matlab
build_furuta_pid
verify_furuta_pid
open_system(fullfile('models','furuta_pid_example.slx'))
```

Press Run to simulate a five-degree initial pendulum displacement and a one-volt disturbance pulse from 4.0 to 4.1 seconds. The Angle Scope shows the arm and pendulum angles in radians. The Voltage Scope shows the applied motor terminal voltage. Results contain the four plant states, applied voltage and controller voltage before limiting.

The model embeds its coefficients in its model workspace. Open PID Controller and then PID Function to inspect the controller code. The continuous controller state is stored in an external Integrator block. The reusable MATLAB implementation is [furuta_pid.m](../src/controllers/furuta_pid.m). The model embeds a copy of this function and its gains when built. Rebuild after changing either file. Open Plant Dynamics to inspect the nonlinear equations.

## Control law

The reference for both angles is zero, with theta1 = 0 at upright. The controller combines arm-position PID and pendulum-angle PD feedback:

```text
u_raw = armP*theta0 + integralVoltage + armD*filteredOmega0
        + pendulumP*theta1 + pendulumD*filteredOmega1

integralVoltage_dot = armI*theta0 + 5*(u_limited - u_raw)
filteredOmega = 80*s/(s+80)*theta
u_limited = clamp(u_raw, -12, 12) volts
```

Each derivative filter starts at the initial angle to avoid an artificial derivative kick. The model applies a second voltage limit after adding the disturbance.

Example coefficients are armP = 1.65936339, armI = 0.84980173, armD = 1.45926708, pendulumP = 38.21712926 and pendulumD = 4.83239446. The code calculates full-precision values. Angles are radians, time is seconds and output is volts. These signs follow the plant's angle convention. With a conventional reference-minus-measurement error, the corresponding P, I and D gains are negative.

Initial coefficients are synthesized from the upright linearization and an arm-angle integrator using Ackermann's formula with target poles [-1.5 -2 -3 -4 -5]. Filter states change the actual closed-loop poles, which the verification checks separately. This is an initial classical PID/PD controller design. Its gain synthesis uses the linear model and can be retuned to specific hardware performance requirements.

## Verification and interpretation

The verification runs two nonlinear experiments, a five-degree release from upright and a one-volt motor-voltage pulse lasting 0.1 seconds from an initially upright state. It checks recovery, voltage bounds, filtered closed-loop eigenvalues, and agreement with an independent MATLAB ODE integration.

CSV traces, response plots and a metrics table are saved under `output/pid/`. Recovery is the time from release or pulse end until both angles remain within 0.5 degree for the pendulum and 2 degrees for the arm. The metrics also include peak angles, voltage peak/RMS and pendulum IAE. These tests cover the example plant and the specified small disturbances.

The 12 V limit is a simulation assumption. Replace it with the actual supply and driver limits. Physical parameters, mounting geometry, sample time, sensor filtering and signs require confirmation before hardware operation. The model starts near upright. Swing-up and Stateflow switching are subsequent integration tasks.

Once all measured parameters are available, use `build_furuta_pid("measured")` and `verify_furuta_pid("measured")` to regenerate the coefficients and check that model.
