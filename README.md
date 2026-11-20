# Furuta pendulum project

The MATLAB project includes a plant model for Section 4.1 and dashboard and digital twin work for Sections 4.5 and 4.6. The dashboard displays synthetic signals and animates a synchronized pendulum schematic. The controller models use the shared L298N, motor, and CAD-derived Simscape plant for wiring checkout. Controller tuning, dashboard integration, and hardware testing remain pending.

## Run and verify

Open this folder in MATLAB R2026b. Run `run_dashboard` to launch the demo or `verify_dashboard` to run the smoke test. Both scripts resolve project paths from their own location.

| Area | Files |
| --- | --- |
| MATLAB entry points | [Launch the dashboard](run_dashboard.m) · [Run verification](verify_dashboard.m) |
| Plant model | [Plant setup](documentation/plant-model.md) · [Euler-Lagrange derivation](documentation/nonlinear-derivation.md) · [Parameter table](src/plant/furuta_parameters.m) · [Example plant builder](build_furuta_plant.m) |
| Simscape Multibody plant | [CAD mechanical plant and validation](documentation/multibody-plant.md) · [Multibody model](models/furuta_multibody.slx) · [Open detailed CAD animation](view_furuta_multibody.m) |
| Voltage-driven Simscape baseline (generated on demand) | [Voltage-profile verification](verify_furuta_voltage_plant.m) · [Upright linearization verification](verify_furuta_voltage_upright.m) |
| L298N-driven Simscape plant | [Driver, motor and plant model](models/furuta_multibody_l298n.slx) · [Driver verification](verify_furuta_l298n.m) |
| Switching electrical drive | [Simscape Electrical motor and H-bridge](models/furuta_multibody_electrical.slx) · [Switching verification](verify_furuta_electrical_drive.m) |
| Switching electrical drive with swing-up | [Electrical swing-up model](models/furuta_multibody_electrical_swingup.slx) · [Swing-up verification](verify_furuta_electrical_swingup.m) |
| CAD controller wiring | [Models and wiring verification](documentation/cad-controller-wiring.md) |
| PID controller | [Run, gains and validation](documentation/pid-controller.md) · [CAD wiring model](models/furuta_pid_model.slx) |
| Pole-placement controller | [Design and validation](documentation/pole-placement-controller.md) · [CAD wiring model](models/furuta_pole_placement_model.slx) |
| LQR controller | [Weights, design and validation](documentation/lqr-controller.md) · [CAD wiring model](models/furuta_lqr_model.slx) |
| Swing-up and Stateflow | [Operation and switching guards](documentation/hybrid-control.md) · [CAD wiring model](models/furuta_hybrid_model.slx) |
| Application | [Dashboard folder](dashboard/) · [Dashboard class](dashboard/FurutaDashboard.m) |
| Course requirements | [Official Fall 2026 project brief](documentation/description/furuta-pendulum-project-brief-fall-2026.pdf) |
| Dashboard integration | [Signal interface, units, and CSV logging](dashboard/README.md) |
| Motor information | [GA25-370 supplied specifications](documentation/motor-specifications.md) |
| CAD inputs | [Inventor review](documentation/cad-review.md) · [CAD file guide](cad/README.md) · [Inventor source package](cad/originals/modern%20control/) |
| Research reference | [Akhtaruzzaman and Shafie (2010): modeling and controller comparison](documentation/references/2010_ICMA_ModelingandControlofaRotaryInvertedPendulumUsingVariousMethods.pdf) |
| Motor identification reference | [DC motor parameter estimation tutorial](parameter%20estimation/) |
| Incoming files | [Drop new files here](resources/) |
| Verification | [Dashboard smoke test](dashboard/test_dashboard.m) |
| Generated results | [Interactive project checklist](output/pdf/furuta-project-checklist.pdf) · [Output guide](output/README.md) · [Dashboard preview](dashboard/output/dashboard_preview.png) · [Sample CSV](dashboard/output/dashboard_test.csv) · [Saved MATLAB log](dashboard/output/dashboard_test.log) |

## Integration next steps

1. Confirm the pendulum mounting and joint geometry on the physical assembly.
2. Record dimensions, CAD units, pivot locations, and angle conventions.
3. Replace the schematic with the CAD geometry.
4. Connect the hardware angle signals, motor command, and Stateflow mode.
5. Verify that the twin and dashboard stay synchronized with the physical system.


