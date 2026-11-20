# Synthetic GA25-370 example readings

[Download the CSV](example_motor_readings_synthetic.csv). This data was generated for practice. It contains no measurements from a physical motor. The model was chosen to reproduce the project's reported 12 V and 200 RPM no-load operating point. The quoted 4.5 kgf cm torque has an unspecified operating condition and was not used. The actual gear ratio and motor constants remain unverified.

The log contains 1,126 samples spaced 0.02 s apart, with forward and reverse voltage steps and zero-voltage intervals. `voltage_V` is the signed input held until the next sample. `speed_rpm` is signed gearbox output speed. `current_A` is signed winding current. Independent zero-mean noise was added to the speed and current channels with standard deviations of 0.65 RPM and 0.008 A, respectively. Zero-voltage intervals follow the same electrical model as the rest of the run and can include braking current.

The generator uses an equivalent gearbox-output model with no external load. With output speed `omega` in rad/s, `i = (V - Ke*omega)/R` and `J*domega/dt = Kt*i - B*omega`. It assumes negligible winding inductance and models friction as viscous damping. `Ke` and `Kt` are effective output-side coefficients.

| Generator parameter | Practice value |
| --- | ---: |
| Terminal resistance `R` | 6.0 ohm |
| Effective back-EMF coefficient `Ke` | 0.53571554 V/(rad/s) |
| Effective torque coefficient `Kt` | 0.15 N m/A |
| Equivalent inertia `J` | 0.00286479 kg m^2 |
| Equivalent viscous damping `B` | 0.00093106 N m/(rad/s) |

Import the CSV with MATLAB `readtable`. Use voltage as the input and RPM as the measured output for the tutorial model. Use the current channel if the fitting model also exposes current. This data can exercise fitting of the effective electrical and speed-response relationships. With no applied load torque, the speed and current traces cannot separate `Kt`, `J`, and `B` individually: scaling all three together gives the same response. The 50 Hz log and its simplified model also provide no basis for estimating winding inductance.

Run `fit_example_motor` from this folder to reproduce the fit in base MATLAB. It estimates terminal resistance, effective back-EMF, steady-state speed gain, and the speed response time constant. The fit uses voltage, RPM, and current before 17.5 s. It predicts the final +6 V pulse and decay from 19 s onward without using those speed or current readings as fitting inputs. The `example fit results` folder contains the fitted coefficients and held-out predictions.

These coefficients characterize an unloaded speed and current model for the practice data. A measured load or independent inertia/torque information is needed to estimate an absolute shaft-torque model from this experiment design.

Run `fit_example_friction_gearbox` to reproduce the separate friction and gearbox practice experiments. The arm coast-down trial holds the pendulum at hanging and assumes zero applied shaft torque. The hinge free-swing trial holds the arm fixed and logs the angle relative to hanging. Both fits use the CAD inertias and compare predictions against a second synthetic release. Their fitted coefficients represent viscous friction under the simulated conditions.

The gearbox example fits the motor-to-output speed ratio and mechanical efficiency from simultaneous speed and torque at both shafts. It uses six synthetic steady operating points for fitting and three for validation. The generator uses a hypothetical 30:1 gearbox with 75% efficiency in the tested direction. These values are independent of the unloaded motor practice fit. The results and trial data are in `example fit results`. Measuring a real gearbox ratio requires the actual variant's specification or both shaft speeds or turn counts. Measuring mechanical efficiency requires input and output shaft power under load.
