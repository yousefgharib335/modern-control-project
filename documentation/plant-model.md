# Furuta pendulum plant model

The [CAD-derived Simscape Multibody plant](multibody-plant.md) is a separate mechanical model with generalized rigid-body equations and its own upright state-space matrices. Its custom CAD inertia tensors and selected moving groups differ from the earlier example parameters.

## Run the example

From the project folder in MATLAB, run:

```matlab
build_furuta_plant
verify_furuta_plant
open_system(fullfile('models','furuta_plant_example.slx'))
```

The Simulink plant accepts effective motor terminal voltage in volts. It outputs the state vector `[theta0; theta1; omega0; omega1]` in radians and radians per second. The initial condition is a 0.1 rad displacement from hanging with zero speed. The example values are illustrative. Open the Dynamics subsystem to inspect the DC motor, mass matrix, generalized forces, and acceleration solve. These use standard Simulink blocks. The model workspace contains the coefficients from the parameter set used to build the model.

The complete [Euler-Lagrange derivation](nonlinear-derivation.md) develops the kinematics, energies, both joint equations and the voltage-driven motor model.

## Model and conventions

The positive arm angle rotates from the global X axis toward Y. With the arm on X, a positive pendulum displacement from upright moves the pendulum toward Y. In the ideal geometry the arm pivot is at `L0*[cos(theta0); sin(theta0); 0]` and the pendulum's center of mass is offset by `l1*[-sin(theta0)*sin(theta1); cos(theta0)*sin(theta1); cos(theta1)]`.

Let `Jp = J1cm + m1*l1^2` and `d = Jp - J1ax`. The total kinetic energy is `0.5*qdot'*M(q)*qdot`, where `M11 = J0 + m1*L0^2 + Jp*sin(theta1)^2 + J1ax*cos(theta1)^2`, `M22 = Jp`, and `M12 = M21 = m1*L0*l1*cos(theta1)`. Potential energy is `m1*g*l1*cos(theta1)`. Applying Euler–Lagrange gives `M*qddot + h + G + diag(b0,b1)*qdot = [torque; 0]`. The velocity terms `h` and gravity terms `G` are implemented in [furuta_dynamics.m](../src/plant/furuta_dynamics.m).

The links are rigid, the pendulum axes are principal axes, and joint friction is viscous. The DC motor uses `i = (voltage - Ke*N*omega0)/R` and `torque = eta*N*Kt*i`. This approximation neglects inductance, backlash, stiction and saturation. At zero effective terminal voltage the model includes electrical braking. A disconnected motor can produce a different response.

## Finish Phase 1 with team measurements

1. Fill the measured column in [furuta_parameters.m](../src/plant/furuta_parameters.m) and record actual measurement methods. Record encoder signs, upright offset and the driver command-to-voltage scaling.
2. Run `build_furuta_plant("measured")` followed by `verify_furuta_plant("measured")`. Rebuild the SLX whenever measured values change because the model embeds a parameter snapshot.
3. Record time and both encoder angles during a small release near the hanging position. Match the model input to the actual driver's brake or coast mode. Use the same initial angles and speeds, overlay the simulated and measured traces, and report frequency and decay differences.
4. If the team builds the preferred Simscape Multibody model, compare it with the analytical model using identical parameters, inputs and initial conditions.

For upright controller design, run `p = furuta_parameters("measured"); [A,B,C,D] = furuta_linearize(p);` after running the build or verification command. The outputs of the linear state-space model are the two measured angles. Pass `pi` as the second argument for the hanging equilibrium.
