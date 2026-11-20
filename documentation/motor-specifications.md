# Motor information supplied on 18 November 2026

| Property | Supplied value | SI conversion or status |
| --- | --- | --- |
| Model | GA25-370 DC gear motor with encoder | Exact manufacturer/variant pending |
| Supply | 12 V DC | Nominal supply |
| No-load output speed | 200 RPM | 20.943951 rad/s |
| Output torque | 4.5 kgf cm | 0.441299 N m; rated versus stall classification unspecified |
| Gears | Metal | Provisional signed ratio listed below |
| Output shaft | Approximately 6 mm | Verify against purchased unit |
| Encoder | Built in | Pulse count, quadrature convention and signal levels pending |

These are user-supplied product details. The 4.5 kgf cm figure is recorded as an output-torque specification with an unspecified operating condition.

## Additional provisional motor assumptions supplied on 20 November 2026

| Parameter | Assumed value | Model interpretation |
| --- | --- | --- |
| Signed gearbox ratio | −35.0048 | Stored as the supplied signed ratio. The motor coordinate in the model is oriented to match the arm, so its speed and torque conversions use the reduction magnitude, 35.0048:1. |
| Motor-shaft viscous friction | 4.08 × 10⁻⁹ N·m·s/rad | Multiplying by the squared ratio gives 4.99937 × 10⁻⁶ N·m·s/rad at the output. This is the drive viscous loss used in the model. The independent arm and pendulum joint damping estimates remain separate. |
| Motor armature inductance | 12.003374 µH | 1.2003374 × 10⁻⁵ H for the Simscape DC Motor block. |

These three values are user-supplied assumptions from the screenshot. Their measurement basis and shaft sign convention still need confirmation. Resistance, effective output-side back EMF and torque constants, gear efficiency, and reflected inertia retain their earlier provisional values. Keeping the effective output constants preserves the existing voltage and torque sign across the new shaft conversion.

## Voltage-driven simulation

Run `build_furuta_multibody("voltage")` to generate `models/furuta_multibody_voltage.slx`. Open it to see the **DC Motor and Gearbox** connected to the **Mechanical Plant**. The motor limits the voltage command to ±12 V and sends drive torque to the arm joint. The plant feeds arm speed back to the motor for back EMF and viscous loss. The top-level outputs are the four mechanical states, motor current, and applied drive torque. Open **Mechanical Plant** to see the CAD bodies, Simscape joints, and four-panel State Scope. Run `verify_furuta_voltage_plant` to test a forward and reverse voltage profile. The verification script generates the model if the file is missing.

| Simulation parameter | Value | Basis |
| --- | --- | --- |
| Winding resistance | 6.0026 Ω | Synthetic motor fit |
| Effective output-side back-EMF constant | 0.53578 V/(rad/s) | Synthetic motor fit |
| Effective output-side torque constant | 0.15 N·m/A | Assumed in the synthetic data |
| Motor-to-output speed ratio | 35.0048:1 | Supplied ratio magnitude |
| Gear efficiency | 74.57% | Synthetic gearbox fit |
| Reflected drive inertia | 0.0028644 kg·m² | Derived from the fitted speed time constant |
| Output-side drive viscous damping | 4.99937 × 10⁻⁶ N·m·s/rad | Supplied motor friction reflected through the gearbox |
| Arm and pendulum hinge damping | 0.0001100 and 0.00002404 N·m/(rad/s) | Synthetic joint friction fits |

The model converts the effective constants to motor-side gains before applying the explicit gearbox gains. This accounts for the gear ratio and efficiency once. The torque output is the geared electromagnetic torque minus drive viscous loss. The attached Simscape Inertia block represents the reflected rotor and gearbox inertia at the arm joint. The winding current is algebraic, with inductance neglected. The gearbox uses a constant efficiency factor, and the model uses ideal joint-rate sensing.

The original synthetic fit predicts approximately 200 RPM at 12 V for the unloaded drive with its original damping. The 4.5 kgf cm product figure has an unspecified operating condition and is recorded separately from the model's assumptions. Motor and gearbox identification, driver current limits, and encoder resolution still need hardware data. The [controller models](cad-controller-wiring.md) use the switching electrical variant described below.

## L298N driver simulation

Open [furuta_multibody_l298n.slx](../models/furuta_multibody_l298n.slx) to see the selected driver in front of the motor and mechanical plant. The inputs are **Motor Voltage V** and **Brake Command**. With brake released, the sign of the command selects direction, and its magnitude divided by 12 V sets enable-PWM duty, limited to 100%. A zero command disables the bridge and coasts. A brake command of 1 overrides the voltage request and models braking by shorting the motor terminals.

The driver model uses a fixed 2.55 V drop across the conducting bridge, based on the sum of the typical source and sink drops at 1 A in Table 4. This gives a 9.45 V on-state voltage from a nominal 12 V supply. The approximation uses zero winding current in disabled intervals and ideal terminal shorting during brake. It omits switching transients, diode recirculation, brake-path losses, and current and thermal protection. The simulated average current does not establish the actual module's peak current or thermal margin. [ST L298 datasheet, Tables 1, 4 and 5](https://www.st.com/resource/en/datasheet/l298.pdf)

The original voltage-input state-space model describes the ideal voltage interface. The L298N model reduces the local command-to-torque gain and coasting removes electrical damping at zero duty. Use the L298N dynamics for controller designs intended for this driver.

Run `verify_furuta_l298n` to exercise forward, reverse, half duty, saturation, coast, and brake. Rebuild with `build_furuta_multibody("l298n")`.

## Switching electrical model

Open [furuta_multibody_electrical.slx](../models/furuta_multibody_electrical.slx) and double-click **System** to see **H-Bridge**, **DC Motor**, and **Pendulum Plant**. Two Simscape electrical terminals connect the bridge to the motor. Use **Motor Current A** and **Motor Terminal Voltage V** to inspect individual PWM pulses. The motor receives arm speed multiplied by the gear ratio. To preserve the independently assumed effective drivetrain torque constant, the circuit's sensed current is converted to fitted output torque and sent to the arm joint. The reflected drive inertia acts through the pendulum plant once.

The armature inductance is provisionally **12.003374 µH** and the PWM frequency is provisionally **1 kHz**. The generic bridge uses a **9.45 V** internal amplitude based on the nominal 12 V supply minus the estimated L298N drop at 1 A. It uses two freewheeling diodes during PWM off intervals. The circuit omits board-specific thermal behavior and overcurrent protection. Calibrate inductance, PWM frequency, and driver losses before interpreting peak currents as hardware predictions. [MathWorks H-Bridge documentation](https://www.mathworks.com/help/simscape-electrical/ref/hbridge.html)

In the base electrical model, **Motor Voltage V** drives the H-bridge directly. The [electrical model with swing-up](../models/furuta_multibody_electrical_swingup.slx) starts hanging at rest. Its **Drive Command** selects **SwingUp** by default and sends the provisional controller command to the H-bridge. Set its active choice to **Manual** to use **Motor Voltage V** directly. The hybrid balance controller and Stateflow supervisor are in the separate [hybrid controller model](../models/furuta_hybrid_model.slx), which uses the same electrical **System**.

Run `verify_furuta_electrical_swingup` to check the swing-up command path and `verify_furuta_electrical_drive` for the manual switching test. Rebuild the base or swing-up model with `build_furuta_multibody("electrical")` or `build_furuta_multibody("electrical_swingup")`, respectively.

The block covers use the project CAD render, a representative L298N board photo from [Addicore](https://www.addicore.com/products/l298n-dual-motor-driver-board), and a GA25-370 motor photo from [Most Electronics](https://mostelectronic.com/shop/motors-drives-2/dc-motors-motors-drives-2/ga25-370-210rpm-encoder-motor/). The two source photos are in `images`. The fitted cover images are saved inside the Simulink model.
