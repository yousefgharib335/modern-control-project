# CAD-derived Simscape Multibody plant

Open the project folder in MATLAB and run:

```matlab
build_furuta_multibody
addpath('open loop validation')
verify_furuta_multibody
view_furuta_multibody
```

`view_furuta_multibody` opens the block diagram and Multibody Explorer and simulates release from 0.1 rad away from hanging. The root input is arm torque in N m. The output is [theta0;theta1;omega0;omega1] in radians and radians per second. The State Scope shows each state in a separate plot, with one column for each joint. Multibody Explorer displays the detailed component surfaces and joint motion. The builder disables automatic visualization during numerical verification. The viewer reenables it for interactive use.

For the Milestone 1 fall from upright, run `demo_furuta_milestone1`. It opens the standalone animation and four-panel State Scope, releases the pendulum from 1 degree at zero motor torque, and saves the angle and rate traces. The release uses the example viscous damping from the synthetic estimation exercises. Run `demo_furuta_milestone1(false)` to collect the same numerical results without opening either window. The [response plot](../output/multibody/milestone1/fall_from_upright.png) shows the fall, oscillations, and settling about 180 degrees. The check defines settling as staying within 1 degree of hanging and 1 degree per second until the end of the run, with at least two seconds in that band.

## Physical model and grouping

The plant uses eight Simscape Multibody File Solid blocks and seven detailed STL meshes. Two stationary aluminum profiles share one mesh. Each mesh retains its source-local frame and is explicitly interpreted in meters. Complete custom CAD masses, COM locations and inertia tensors define the rigid bodies. Two Revolute Joint blocks provide the driven vertical arm rotation and free radial pendulum hinge. Rigid Transform blocks place the component frames. World Frame, Mechanism Configuration and Solver Configuration establish the physical network and gravity. The model does not simulate contact with the base.

The component grouping is an implementation decision based on the current assembly and shaft mates:

| Group | CAD occurrences |
| --- | --- |
| Fixed base | Motor:1, motor holder:1 and both grounded aluminum profile occurrences |
| Rotating arm | Assembly2:1, including its case and coupling, and ROTERY ENCODER.step:2 |
| Swinging pendulum | Rigid 6 to 8mm Coupler v2:1 and ALLEN_04_mm:1 |

The unmounted ROTERY ENCODER.step:1 is omitted. Two unresolved source mates inside Assembly2 are replaced in the simulation by an explicit rigid grouping at the supplied CAD pose. This model construction defines the simulation connections. The supplied Inventor source assembly retains its original constraints.

The rotating arm group has mass 0.388242408 kg and vertical-axis inertia 8.61464773e-5 kg m^2 about the driven axis. The swinging group has mass 0.050766148 kg. Its hinge-to-COM perpendicular distance is 0.014643230 m. The revision changes some moving part placements in addition to adding the stationary base, so the moving COM and inertia tensors were recomputed from the revised occurrence transforms.

The pendulum hinge reference is at [0.06925;0;-0.039] m relative to the local arm frame. The composite pendulum COM at upright is [-0.026705195;0;-0.014643230] m relative to that hinge in the local arm frame. Local negative Z points up in world coordinates after the mounting rotation. The radial COM offset and the full inertia tensor are retained.

All these quantities follow the CAD material assignments. The encoder's exported body is treated as fixed to the arm. Its internal rotor has no separate supplied inertia model. Motor rotor/gearbox inertia and friction default to zero in this initial mechanical model. These assumptions require later identification.

## Coordinates, torque and initialization

The Assembly Mounting Frame rotates the entire mechanism 180 degrees about its local X axis. The world gravity vector remains [0;0;-g]. The arm frame is on the driven motor axis. Its local X axis points outward along the hinge, and its local positive Z points world-down. Positive theta0 follows the right-hand rule about this local axis, appearing clockwise from above. Positive torque uses the same axis as theta0.

The pendulum joint's positive axis is negative arm X. At theta1 = 0 the composite pendulum COM is vertically above the hinge axis. At theta1 = pi it hangs below the axis. This zero uses the combined swinging group. The long Allen leg can therefore have a small offset from the exact COM-upright direction.

The model workspace contains `p` and `initialState`. Change the latter to set angles and rates at simulation start. The default is [0;pi+0.1;0;0]. Joint state targets specify initial conditions while the joint motions evolve dynamically.

A Simulink-PS Converter applies the torque to the arm joint. Four PS-Simulink Converters expose joint position and angular rate. Motor voltage and current are subsequent additions to this torque-driven mechanical plant. The supplied GA25-370 specifications are recorded in [motor-specifications.md](motor-specifications.md). They give a nominal 12 V supply, 200 RPM no-load speed and a quoted 0.441299 N m torque whose operating classification remains unspecified.

## Matching analytical rigid-body model

[furuta_cad_dynamics.m](../src/plant/furuta_cad_dynamics.m) uses the same composite mass properties without assuming a symmetric straight pendulum. Let a = [-1;0;0], z = [0;0;1], h be the hinge position and r0 be the pendulum COM offset at upright.

```text
R = rotation about a by theta1
rP = R*r0
r = h+rP
I = R*I0*R'
v0 = cross(z,r)
v1 = cross(a,rP)

M11 = Jarm + m*(v0'*v0) + z'*I*z
M12 = m*(v0'*v1) + z'*I*a
M22 = m*(v1'*v1) + a'*I*a
```

Here Jarm includes the arm group's COM tensor and its parallel-axis contribution about the motor axis. The kinetic energy is T = 0.5*qdot'*M*qdot. The gravitational potential is V = m*g*(worldUp'*r), with worldUp = [0;0;-1] expressed in the mounted arm frame.

Only theta1 changes M. With a prime denoting differentiation with respect to theta1, Euler-Lagrange gives the velocity terms h0 = M11'*omega0*omega1 + M12'*omega1^2 and h1 = -0.5*M11'*omega0^2 + 0.5*M22'*omega1^2. The gravity vector is [0; m*g*worldUp'*r']. Viscous joint friction, when specified, adds diag(b0,b1)*qdot.

The analytical function solves M*qddot = [torque;0] - h - G - friction. It also calculates mechanical energy for conservation and dissipation checks.

## Validation

The verification covers zero-torque hanging motion, a constant torque near hanging, small upright perturbations and a separate nonzero-viscous-damping case. It compares all four Simscape states with an independent MATLAB ODE integration of the CAD equations. It checks initial conditions, angle signs, units, conservative energy balance and damping energy loss.

It also obtains A, B, C and D at upright and compares the nonlinear mechanical model with the linear state-space response over a short interval with a 0.001 rad initial displacement. These matrices accept torque in N m.

Verification writes traces, error metrics and CAD state-space matrices under `output/multibody/`. Response overlays of the verified run are saved there as PNG files. These checks establish consistency for the selected CAD grouping and assumed friction. Physical parameter validation remains separate.

The verifier opens two figures by default. One overlays the Simscape and nonlinear pendulum angle in all four tests. The other compares both angles and both angular rates from the nonlinear and linearized models near upright. Run `verify_furuta_multibody(false)` to save the numerical results without opening the figures.

The [controller wiring models](cad-controller-wiring.md) use the CAD plant with the [L298N driver and motor](motor-specifications.md). The torque-input model remains available for open-loop mechanical validation.

## Source files and rebuilding

[plant-properties.json](../cad/simscape-exports/plant-properties.json) stores the grouping, frame transformations and SI mass properties. The [CAD review](cad-review.md) links both assembly revisions and describes the constraint issues. The seven STL meshes are in `cad/simscape-exports/`.

Re-extract the CAD properties after changing the Inventor design or materials. Update the grouping and SI parameter snapshot. Run `export_furuta_meshes.ps1` with Windows PowerShell and Inventor installed to refresh the binary STL surfaces at high resolution in meters, then rebuild the model. The saved File Solid model contains its physical parameters in the model workspace and references the exported meshes. Keep the export folder with the model. Rebuild after relocating the project to refresh its absolute geometry paths.

The block conventions follow [MathWorks custom-inertia documentation](https://www.mathworks.com/help/sm/ug/specify-custom-inertia.html). Geometry exports use the Autodesk translators with [documented STL settings](https://help.autodesk.com/cloudhelp/2025/ENU/Inventor-API/files/TranslatorSettings.htm).

## Verified results

| Experiment | Maximum angle mismatch (rad) | Maximum rate mismatch (rad/s) |
| --- | --- | --- |
| hanging_free | 8.16e-10 | 1.27e-09 |
| hanging_torque | 4.25e-10 | 5.11e-09 |
| upright_small | 8.47e-10 | 1.03e-08 |
| hanging_damped | 3.67e-10 | 4.55e-09 |

All four comparisons passed, together with conservative energy, viscous dissipation and initial-state checks. The short-interval upright linear/nonlinear comparison had a maximum state error of 7.48e-9 in the respective SI state units.

The CAD torque-input linear model has controllability rank 4 and observability rank 4 for outputs [theta0;theta1]. The upright poles are approximately 0, 0, 12.5807, and -12.5807.

The [hanging equilibrium analysis](hanging-equilibrium.md) gives the corresponding state-space model at 180 degrees, with and without the demo damping.

The [validation overview](../output/multibody/validation_overview.png) overlays the Simscape and analytical pendulum angles. The [small-signal comparison](../output/multibody/upright_linear_comparison.png) shows the nonlinear and upright state-space responses.

Load the saved matrices with:

```matlab
load(fullfile('output','multibody','cad_state_space.mat'),'A','B','C','D')
```

## Voltage-driven upright model

The operating point is `x = [0;0;0;0]` and `V = 0`, with state order `[theta0; theta1; omega0; omega1]` and `theta1 = 0` upright. The motor torque near this point is `(Kt/R)*V - (Kt*Ke/R + Bdrive)*omega0`, with the effective output-side motor constants. The ±12 V limiter has unit slope at zero. The linearization uses the reflected drive inertia and both joint damping values from the voltage-driven plant.

Run `verify_furuta_voltage_upright` to compare the state-space response against Simscape for the equilibrium, a small initial angle, and a small voltage step. It saves the four-state voltage-input model and validation traces under `output/multibody`.

```matlab
load(fullfile('output','multibody','voltage_upright_state_space.mat'),'A','B','C','D')
```

The input is in volts. The outputs are both angles in radians followed by both angular rates in rad/s.
