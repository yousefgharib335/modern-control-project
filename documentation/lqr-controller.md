# LQR upright stabilization

From the project folder in MATLAB, run:

```matlab
build_furuta_lqr
verify_furuta_lqr
open_system(fullfile('models','furuta_lqr_example.slx'))
```

Press Run to start from a five-degree pendulum displacement. The model also applies a one-volt motor disturbance from 4.0 to 4.1 seconds. Open LQR Controller to inspect its MATLAB Function. The State Scope displays theta0, theta1, omega0 and omega1 in radians and radians/second. The Voltage Scope displays applied motor terminal voltage.

## Design and weights

The state vector is x = [theta0;theta1;omega0;omega1], with both target angles at zero and theta1 = 0 at upright. LQR selects full-state feedback u = -K*x to minimize the continuous-time infinite-horizon cost:

```text
J = integral(x'*Q*x + u'*R*u) dt
```

Bryson scaling supplies an interpretable starting point:

| Quantity | Tuning scale |
| --- | --- |
| Arm angle | 20 deg |
| Pendulum angle | 5 deg |
| Arm and pendulum angular rates | 2 rad/s each |
| Motor voltage | 6 V |

Q = diag([1/deg2rad(20)^2, 1/deg2rad(5)^2, 1/2^2, 1/2^2]) and R = 1/6^2. A displacement equal to any one listed scale contributes one unit of cost per second. The smaller pendulum angle scale weights balancing more strongly. The scales are cost weights. The separate simulated actuator saturation is plus/minus 12 V.

The design solves the continuous algebraic Riccati equation A'*S + S*A - S*B/R*B'*S + Q = 0 and computes K = R\(B'*S). It uses the stable invariant subspace of the Hamiltonian matrix, allowing this small model to run without Control System Toolbox. The implementation checks controllability, the Riccati residual, positive definiteness of S, and negative real parts of all closed-loop poles. Verification also checks the LQR value-function identity.

Retrieve the weights, gain and poles with:

```matlab
addpath('src/plant','src/controllers')
design = furuta_lqr_design(furuta_parameters("example"));
design.Q
design.R
design.K
design.actualPoles
```

To increase the cost of pendulum deviations, reduce its angle scale. To increase the penalty on motor voltage, reduce the voltage scale. Rebuild and verify after changing weights or plant parameters. The model embeds its gain matrix when built.

## Verification and remaining work

The nonlinear tests use the same five-degree initial release and one-volt, 0.1-second pulse as the other controllers. Verification checks upright recovery, voltage limits and agreement with an independent MATLAB ODE simulation. Output traces, plots and metrics are saved under `output/lqr/`.

Recovery is measured until both angles stay within 0.5 degree for the pendulum and 2 degrees for the arm. The metrics also report peak angles, peak/RMS voltage and pendulum IAE.

LQR optimality applies to the specified linear model and quadratic cost with unconstrained input. The nonlinear experiments check performance with the chosen saturation. Hardware parameters and rate estimates still need to be supplied, and the example starts close to upright. A constant load can cause an offset because this design has no integral augmentation.

When measured parameters are complete, run `build_furuta_lqr("measured")` and `verify_furuta_lqr("measured")`. Physical testing, robustness experiments, swing-up and Stateflow switching remain subsequent work.

## Example results

The verified example gain is approximately K = [-17.1887 -156.4466 -10.6065 -19.6810] in u = -K*x. The closed-loop poles are approximately -16.8359 +/- 7.0726i and -2.7370 +/- 1.8511i per second. The normalized Riccati residual was 5.34e-15.

| Experiment | Recovery within the stated angle tolerances | Peak arm angle | Peak motor voltage |
| --- | --- | --- | --- |
| Five-degree initial pendulum displacement | 0.951 s | 9.686 deg | 12.000 V |
| One-volt, 0.1-second motor disturbance | Angles remained within tolerance throughout | 0.569 deg | 1.290 V |

The release test reaches the assumed voltage limit. The disturbance test's reported recovery metric is zero because neither angle leaves the recovery bands. Both tests passed independent ODE agreement and final-angle checks.
