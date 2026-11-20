# Stability with the switching electrical System

The standalone electrical model uses a 1 kHz switching H-bridge, an assumed 12.003374 µH winding inductance, and the CAD pendulum. For the near-upright check, the model starts at zero joint angles and rates, with brake released. Each command is held for eight PWM periods. The verifier averages current over five complete periods after the first two periods of each command.

## Averaged current and operating range

The algebraic average predicts 19.68 mA at 0.15 V. The updated switching model produces about 12.68 mA. At both ±0.03 V the switching current is numerically zero. The verifier fits an odd, shape-preserving interpolant to the paired current magnitudes from 0 to 2.8 V and uses a flat segment from −0.03 to +0.03 V. On the held-out commands ±0.25, ±0.50, ±1, and ±2 V, the algebraic average has an 8.25 mA RMSE and the fitted average has a **2.89 mA RMSE**. This fit describes low-speed, brake-released simulations over the tested range. Hardware deadband requires measurement. [Current comparison](../output/multibody/electrical_upright_pwm_comparison.csv)

## Open-loop equilibria

The four-state model uses the same rigid-body equations, reflected drive inertia, and assumed viscous friction as the electrical System. It averages out the winding current, sets its local command derivative to zero from the fitted flat region, and uses the two joint angles as outputs. The pole units are s⁻¹.

| Equilibrium | Poles | Controllability rank | Observability rank |
| --- | --- | ---: | ---: |
| Upright | 0, +10.9222, −11.3306, −0.0376 | 0 | 4 |
| Hanging | 0, −0.0376, −0.2042 ± 11.123i | 0 | 4 |

The upright pole at +10.9222 is unstable. The three decaying modes at hanging describe damped pendulum motion and joint rates, with a neutral arm-angle mode. Rank zero is the local voltage-input result for the fitted no-response band. Larger voltage commands move the arm. The finite-amplitude design approximation uses the 82.11 mA mean current divided by the command at 0.7 V, giving controllability rank four for that approximation. [Equilibrium poles](../output/multibody/electrical_equilibrium_poles.csv)

## PID design

PID uses the 0.7 V finite-amplitude approximation and includes both derivative filters in the pole calculation. With the new motor assumptions, the first recalculated design recovered in 4.236 s, beyond the four-second test budget. The revised target poles are −3, −3.75, −4.5, −5.25, and −6 s⁻¹. The two filter-related poles are near −57.95 and −80 s⁻¹. The updated gains are `(Kp, Ki, Kd) = (1.926, 1.622, 0.923)` for the arm and `(Kp, Kd) = (−58.227, −4.326)` for the pendulum. Inside the fitted no-response band the uncompensated plant keeps the +10.92 s⁻¹ pole because an infinitesimal controller command has zero first-order effect. Classical gain and phase margins for the finite-amplitude approximation describe the compensated averaged design.

In the earlier 0.5 mH simulation before compensation, the release from +1° stayed within ±1° over 2.5 s and ended at −0.177°. The arm was still moving at 0.188 rad/s. Peak requested voltage was 1.916 V. [Earlier uncompensated PID release trace](../output/multibody/electrical_pid_candidate_release.csv)

## Small-command compensation

Each standalone balance controller sends its equivalent voltage request to **Small Command Compensation** once per PWM period. The stage converts the request to an average current target using the 0.7 V fit slope. For larger targets it inverts the measured current map. For targets below the smallest tested nonzero current, it accumulates the fractional demand and schedules signed 0.15 V command pulses among zero-command periods. Braking resets the accumulator. The calibrated controller path caps its bridge request at 2.8 V, the highest command used in the fit. The external test-voltage input still adds a disturbance after compensation.

The average-design poles describe the ideal compensated plant. The switching simulation tests practical recovery in a finite tolerance band. The minimum pulse and the current map come from the simulation assumptions and require hardware calibration before deployment.

## Standalone balance designs

Pole placement uses four distinct real targets at −4, −5, −6 and −7 s⁻¹. The initial targets at −2, −3, −4 and −5 s⁻¹ brought the arm into the settling band at 4.646 s, after the four-second test cutoff. The revised targets reduce the nominal time constant of the slowest mode from 0.5 s to 0.25 s. LQR uses Bryson scaling with 20° for the arm, 5° for the pendulum, 2 rad/s for each rate, and 3 V for the equivalent motor request. The smaller pendulum angle scale gives pendulum error sixteen times the arm-angle weight in radian units. The input weighting penalizes voltage effort within the calibrated command range. Each state-feedback model stores its gain matrix and achieved averaged poles in `stateFeedbackDesign` in its model workspace.

The current pole-placement gain is approximately `[-1.180, 61.966, -0.906, 5.336]`, and the LQR gain is `[-8.594, 139.454, -4.031, 12.074]` for `u = -K*x`, with state order `[arm angle, pendulum angle, arm rate, pendulum rate]`. Their averaged poles are −7, −6, −5, and −4 s⁻¹ for pole placement and −14.39 ± 5.48i and −4.43 ± 2.48i s⁻¹ for LQR. The LQR weights are approximately `Q = diag([8.207, 131.312, 0.25, 0.25])` and `R = 1/9`.

In the original 0.5 mH release test, all three controllers passed the same five-second switching simulation from +1°, with both joints at rest and zero external voltage and brake commands. The settling band is ±0.1° for the pendulum, ±0.5° for the arm, and ±1°/s for each joint rate. The following results use the earlier parameter set.

| Controller | Closed-loop poles (s⁻¹) | Settling time (s) | Peak bridge request (V) |
| --- | --- | ---: | ---: |
| PID | −80, −63.25, −5, −4.25, −3.5, −2.75, −2 | 3.457 | 1.289 |
| Pole placement | −7, −6, −5, −4 | 1.690 | 1.596 |
| LQR | −11.73 ± 2.75i, −3.38 ± 2.47i | 1.298 | 2.165 |

[Earlier release comparison](../output/multibody/electrical_controller_comparison.csv) · [PID trace](../output/multibody/electrical_pid_compensated_release.csv) · [Pole-placement trace](../output/multibody/electrical_pole_placement_compensated_release.csv) · [LQR trace](../output/multibody/electrical_lqr_compensated_release.csv)

## Upright pendulum push test

For this test, the verifier sets the initial state to 0° with zero joint rates. At 0.500 s, two Step blocks generate a 0.0004 N·m pulse at the pendulum joint, ending at 0.550 s. This represents a brief push on the pendulum. Recovery time is measured from the end of the pulse using the same angle and rate bands and four-second recovery budget as the release test. Each controller stayed within the band from its recovery time through the end of the five-second simulation. Interactive runs use the same upright initial state.

| Controller | Peak pendulum tilt (°) | Recovery after push (s) | Peak bridge request (V) |
| --- | ---: | ---: | ---: |
| PID | 1.148 | 2.724 | 1.359 |
| Pole placement | 0.727 | 1.880 | 1.306 |
| LQR | 0.905 | 1.086 | 1.968 |

[Push comparison CSV](../output/multibody/electrical_controller_push_comparison.csv) · [PID trace](../output/multibody/electrical_pid_compensated_push_response.csv) · [Pole-placement trace](../output/multibody/electrical_pole_placement_compensated_push_response.csv) · [LQR trace](../output/multibody/electrical_lqr_compensated_push_response.csv)

To reproduce the original release, call `evaluate_furuta_electrical_controllers(["pid","pole_placement","lqr"],"release")`.

From the project folder in MATLAB, regenerate the analysis and run the upright push test with

```matlab
verify_furuta_electrical_upright
analyze_furuta_electrical_stability
build_furuta_cad_controllers("pid")
build_furuta_cad_controllers("pole_placement")
build_furuta_cad_controllers("lqr")
evaluate_furuta_electrical_controllers
```
