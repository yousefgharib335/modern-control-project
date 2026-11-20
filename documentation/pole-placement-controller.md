# Pole-placement upright stabilization

Run these commands from the project folder in MATLAB:

```matlab
build_furuta_pole_placement
verify_furuta_pole_placement
open_system(fullfile('models','furuta_pole_placement_example.slx'))
```

Press Run to start from a five-degree pendulum displacement. A one-volt disturbance pulse is added at the motor input from 4.0 to 4.1 seconds. The model contains the nonlinear plant and a MATLAB Function controller. The State Scope displays theta0, theta1, omega0 and omega1 in radians and radians/second. The Voltage Scope displays applied motor terminal voltage.

The initial state, disturbance amplitude and design data are stored in the model workspace. The MATLAB Function embeds its gain matrix when the model is built. Rebuild after changing the parameter file or design function.

## Design

The upright equilibrium is x = [0;0;0;0], with state order [theta0;theta1;omega0;omega1]. The input is effective DC motor voltage. The controller is u = -K*x, followed by a voltage limit of plus/minus 12 V.

The design function obtains A and B from the existing upright linearization. It checks that [B A*B A^2*B A^3*B] has rank four, then uses Ackermann's formula to assign closed-loop poles to [-2 -3 -4 -5] per second. These real negative poles provide a nominal, nonoscillatory linear mode set with a slowest time constant of 0.5 seconds. Nonlinear response, modal residues and voltage limits determine the actual angle transients.

The design explicitly checks that the eigenvalues of A-B*K match the requested poles. The gain matrix is printed by verification and can be retrieved with:

```matlab
addpath('src/plant','src/controllers')
design = furuta_pole_placement_design(furuta_parameters("example"));
design.K
design.actualPoles
```

## Validation and assumptions

Verification uses the same initial-release and voltage-pulse cases as the PID experiment. It checks finite trajectories, upright recovery, voltage limits and agreement with a separate MATLAB ODE simulation. CSV traces, response plots and metrics are stored in `output/pole-placement/`.

Recovery time uses the same thresholds as the PID verification. Both angles must remain within 0.5 degree for the pendulum and 2 degrees for the arm after release or pulse end.

This controller uses all four simulated plant states, including exact angular rates. Hardware integration will require rate estimates or an observer. The example parameters and 12 V voltage limit are simulation assumptions. Full measured parameters are still required before hardware tuning. For a measured-parameter design, run `build_furuta_pole_placement("measured")` and `verify_furuta_pole_placement("measured")`.

The model starts near upright. Swing-up and supervisory switching remain subsequent tasks. Plain full-state feedback has no integral correction, so a persistent load or model error can produce an equilibrium offset.

## Example results

The verified example design gives K approximately [-0.5665 -32.1664 -0.9358 -3.9487] in u = -K*x. The saved model uses full-precision gains.

| Experiment | Recovery | Peak arm angle | Peak motor voltage |
| --- | --- | --- | --- |
| Five-degree initial pendulum displacement | 2.684 s | 26.017 deg | 2.807 V |
| One-volt, 0.1-second motor disturbance | 2.051 s after pulse end | 7.497 deg | 1.061 V |

Both experiments passed the nonlinear recovery and independent-integration checks. These results describe the example model.
