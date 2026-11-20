# Nonlinear Furuta pendulum derivation

This derivation defines the idealized two-link model implemented in [furuta_dynamics.m](../src/plant/furuta_dynamics.m) and the Simulink plant. The coordinates, signs and motor approximation below reproduce the current equations.

## Coordinates, parameters and assumptions

The driven arm rotates about a fixed vertical axis with angle theta0. The pendulum rotates about a horizontal radial hinge at the arm tip with angle theta1. Upright is theta1 = 0 and hanging is theta1 = pi. Positive theta0 is counterclockwise from global X toward Y. With the arm on X, a positive theta1 moves the pendulum center of mass toward Y.

| Symbol | Meaning | MATLAB parameter | SI unit |
| --- | --- | --- | --- |
| L0 | Horizontal distance from the driven vertical axis to the pendulum hinge in the ideal geometry | L0 | m |
| l1 | Pendulum hinge to center-of-mass distance along its ideal longitudinal axis | l1 | m |
| m1 | Pendulum moving mass | m1 | kg |
| J0 | Effective inertia of the driven arm and attached arm-fixed components about the vertical axis, including reflected drivetrain inertia | J0 | kg m^2 |
| Jt | Pendulum transverse inertia about its center of mass | J1cm | kg m^2 |
| Ja | Pendulum longitudinal inertia about its center of mass | J1ax | kg m^2 |
| b0, b1 | Arm and pendulum viscous friction coefficients | b0, b1 | N m s/rad |
| g | Gravitational acceleration | g | m/s^2 |
| tau | Torque applied to the arm after the gearbox | computed from voltage | N m |

The mechanical assumptions are rigid links, fixed supports, ideal revolute joints and uniform gravity. The arm remains horizontal. Joint friction is viscous and opposes velocity. Backlash, flexibility, joint clearance, Coulomb friction, stiction and aerodynamic drag are neglected.

The pendulum center of mass lies on its assumed longitudinal axis, and its inertia tensor is axisymmetric, with two equal transverse moments Jt and longitudinal moment Ja. J0 excludes the pendulum terms derived below. A reflected motor rotor inertia N^2*Jmotor can be included in J0 along with arm and gearbox inertia, subject to the chosen drivetrain model.

The motor has constant resistance, torque constant, back-EMF constant, rigid gear ratio and effective gear efficiency. Motor inductance is neglected in the implemented four-state plant. Electrical damping is accounted for through back EMF. The mechanical coefficient b0 excludes that electrical contribution.

These assumptions define an equivalent-link model. The bent Allen key can have an offset center of mass and unequal transverse moments or products of inertia. Its CAD tensor and mounting must be transformed into the chosen joint coordinates to assess this approximation. The scalar mass and length values entered so far do not establish that equivalence.

## Center-of-mass kinematics

Use the orthonormal rotating basis

$$
\mathbf e_r=
\begin{bmatrix}\cos\theta_0\\\sin\theta_0\\0\end{bmatrix},
\qquad
\mathbf e_t=
\begin{bmatrix}-\sin\theta_0\\\cos\theta_0\\0\end{bmatrix},
\qquad
\mathbf e_z=
\begin{bmatrix}0\\0\\1\end{bmatrix}.
$$

Its derivatives are dot(e_r) = dot(theta0)*e_t and dot(e_t) = -dot(theta0)*e_r. Write s = sin(theta1), c = cos(theta1). The pendulum longitudinal unit vector and center-of-mass position are

$$
\mathbf n=s\mathbf e_t+c\mathbf e_z,
\qquad
\mathbf r_C=L_0\mathbf e_r+l_1\mathbf n.
$$

Differentiating gives

$$
\dot{\mathbf r}_C
=-l_1s\dot\theta_0\mathbf e_r
+(L_0\dot\theta_0+l_1c\dot\theta_1)\mathbf e_t
-l_1s\dot\theta_1\mathbf e_z.
$$

Because the three basis vectors are orthonormal,

$$
\dot{\mathbf r}_C^{\mathsf T}\dot{\mathbf r}_C
=(L_0^2+l_1^2s^2)\dot\theta_0^2
+l_1^2\dot\theta_1^2
+2L_0l_1c\dot\theta_0\dot\theta_1.
$$

The pendulum angular velocity is

$$
\boldsymbol\omega_1=\dot\theta_0\mathbf e_z-\dot\theta_1\mathbf e_r.
$$

The minus sign follows from the specified positive pendulum direction. Its longitudinal component is omega1 dot n = dot(theta0)*c, and the squared transverse component is dot(theta1)^2 + dot(theta0)^2*s^2.

## Kinetic and potential energies

The arm kinetic energy is T0 = J0*dot(theta0)^2/2. Pendulum translation and rotation give

$$
T_{1,\mathrm{trans}}=\frac12m_1
\dot{\mathbf r}_C^{\mathsf T}\dot{\mathbf r}_C,
$$

$$
T_{1,\mathrm{rot}}=\frac12J_t
\left(\dot\theta_1^2+s^2\dot\theta_0^2\right)
+\frac12J_ac^2\dot\theta_0^2.
$$

Define

$$
J_p=J_t+m_1l_1^2,
\qquad
d=J_p-J_a,
\qquad
\kappa=m_1L_0l_1,
$$

$$
A(\theta_1)=J_0+m_1L_0^2+J_ps^2+J_ac^2.
$$

The total kinetic energy becomes

$$
\boxed{
T=\frac12A(\theta_1)\dot\theta_0^2
+\kappa c\dot\theta_0\dot\theta_1
+\frac12J_p\dot\theta_1^2
}.
$$

Taking zero potential at the hinge height gives

$$
\boxed{V=m_1gl_1\cos\theta_1}.
$$

The arm contributes a constant gravitational potential. Upright has maximum pendulum potential, consistent with an unstable open-loop equilibrium.

## Euler-Lagrange equation for the driven arm

Set the Lagrangian L = T - V and coordinates q = [theta0; theta1]. With Rayleigh dissipation

$$
\mathcal D=\frac12b_0\dot\theta_0^2+\frac12b_1\dot\theta_1^2,
$$

the equations are

$$
\frac{d}{dt}\frac{\partial L}{\partial\dot q_i}
-\frac{\partial L}{\partial q_i}
+\frac{\partial\mathcal D}{\partial\dot q_i}
=Q_i,
\qquad
\mathbf Q=\begin{bmatrix}\tau\\0\end{bmatrix}.
$$

The kinetic and potential energies are independent of theta0, so partial(L)/partial(theta0) = 0. Also,

$$
\frac{\partial L}{\partial\dot\theta_0}
=A\dot\theta_0+\kappa c\dot\theta_1.
$$

Since dA/dtheta1 = 2*d*s*c,

$$
\frac{d}{dt}\frac{\partial L}{\partial\dot\theta_0}
=A\ddot\theta_0
+2dsc\dot\theta_0\dot\theta_1
+\kappa c\ddot\theta_1
-\kappa s\dot\theta_1^2.
$$

The first nonlinear equation of motion is

$$
\boxed{
A\ddot\theta_0+\kappa c\ddot\theta_1
+2dsc\dot\theta_0\dot\theta_1
-\kappa s\dot\theta_1^2
+b_0\dot\theta_0=\tau
}.
$$

## Euler-Lagrange equation for the free pendulum

The velocity derivative is

$$
\frac{\partial L}{\partial\dot\theta_1}
=\kappa c\dot\theta_0+J_p\dot\theta_1,
$$

$$
\frac{d}{dt}\frac{\partial L}{\partial\dot\theta_1}
=\kappa c\ddot\theta_0
-\kappa s\dot\theta_0\dot\theta_1
+J_p\ddot\theta_1.
$$

The angle derivative is

$$
\frac{\partial L}{\partial\theta_1}
=dsc\dot\theta_0^2
-\kappa s\dot\theta_0\dot\theta_1
+m_1gl_1s.
$$

The mixed velocity terms cancel when these expressions are substituted. The second nonlinear equation is

$$
\boxed{
\kappa c\ddot\theta_0+J_p\ddot\theta_1
-dsc\dot\theta_0^2
-m_1gl_1s+b_1\dot\theta_1=0
}.
$$

The gravity term is negative on the left-hand side because theta1 is measured from upright. Near zero, gravity drives a positive displacement further away from upright.

## Matrix form and explicit state equations

Collect the two equations as

$$
\boxed{
M(q)\ddot q+h(q,\dot q)+G(q)+B_f\dot q
=\begin{bmatrix}\tau\\0\end{bmatrix}
},
$$

$$
M=
\begin{bmatrix}
A&\kappa c\\
\kappa c&J_p
\end{bmatrix},
\qquad
h=
\begin{bmatrix}
2dsc\dot\theta_0\dot\theta_1-\kappa s\dot\theta_1^2\\
-dsc\dot\theta_0^2
\end{bmatrix},
$$

$$
G=
\begin{bmatrix}0\\-m_1gl_1s\end{bmatrix},
\qquad B_f=\operatorname{diag}(b_0,b_1).
$$

Define the net generalized forces

$$
f_0=\tau-b_0\dot\theta_0
-2dsc\dot\theta_0\dot\theta_1+\kappa s\dot\theta_1^2,
$$

$$
f_1=dsc\dot\theta_0^2+m_1gl_1s-b_1\dot\theta_1,
\qquad
\Delta=AJ_p-\kappa^2c^2.
$$

For positive physical inertias the mass matrix is positive definite and Delta is positive. Therefore,

$$
\boxed{
\ddot\theta_0=\frac{J_pf_0-\kappa c f_1}{\Delta},
\qquad
\ddot\theta_1=\frac{Af_1-\kappa c f_0}{\Delta}
}.
$$

With x = [theta0;theta1;omega0;omega1], the four-state nonlinear model is

$$
\dot x=
\begin{bmatrix}
x_3\\x_4\\
(J_pf_0-\kappa c f_1)/\Delta\\
(Af_1-\kappa c f_0)/\Delta
\end{bmatrix}.
$$

The MATLAB implementation solves the two-by-two mass-matrix system directly. Simulink's Acceleration Solve subsystem uses the explicit determinant expressions.

## DC motor and gearbox model

Let u be effective motor terminal voltage, i current, R resistance, Lm inductance, Kt torque constant and Ke back-EMF constant. With gear ratio N = motor speed / arm speed,

$$
\omega_m=N\dot\theta_0,
\qquad
L_m\dot i+Ri+K_eN\dot\theta_0=u.
$$

The implemented approximation sets Lm*dot(i) to zero, giving

$$
i=\frac{u-K_eN\dot\theta_0}{R},
\qquad
\tau=\eta NK_ti.
$$

Consequently,

$$
\boxed{
\tau=\frac{\eta NK_t}{R}u
-\frac{\eta N^2K_tK_e}{R}\dot\theta_0
}.
$$

Define kv = eta*N*Kt/R and be = eta*N^2*Kt*Ke/R. The voltage-driven mechanical model is

$$
\boxed{
M\ddot q+h+G+
\begin{bmatrix}b_0+b_e&0\\0&b_1\end{bmatrix}\dot q
=\begin{bmatrix}k_vu\\0\end{bmatrix}
}.
$$

This shows how back EMF adds arm damping without double-counting it in b0. If electrical transients matter, retain i as a fifth state with dot(i) = (u - R*i - Ke*N*omega0)/Lm and use tau = eta*N*Kt*i in the mechanical equations.

The model assumes a constant effective efficiency in this voltage-to-torque relation, including braking. The motor driver and gear train can require a more detailed quadrant-dependent model. The plant accepts terminal voltage. A PWM duty command needs conversion to effective voltage. Controller saturation is applied outside the mechanical equations. Current limiting and driver dead zones are additional hardware effects to identify.

## Equation checks and implementation mapping

| Derived quantity | Implementation |
| --- | --- |
| Jp, d, kappa | pivotInertia, inertiaDifference, coupling |
| M | mass |
| h | velocityTerms |
| G | gravity |
| Bf*dot(q) | friction |
| eta*N*Kt*(u - Ke*N*omega0)/R | current and torque |
| M inverse times net force | acceleration |

The total mechanical energy H = T + V obeys

$$
\dot H=\tau\dot\theta_0-b_0\dot\theta_0^2-b_1\dot\theta_1^2.
$$

After voltage substitution,

$$
\dot H=k_vu\dot\theta_0-(b_0+b_e)\dot\theta_0^2-b_1\dot\theta_1^2.
$$

This gives a sign check on friction and electrical damping. With zero rates and input, theta1 = 0 and theta1 = pi are equilibria for any constant arm angle. Their upright and hanging Jacobians are implemented in [furuta_linearize.m](../src/plant/furuta_linearize.m) and checked by [verify_furuta_plant.m](../verify_furuta_plant.m).

The derivation is for the stated equivalent-link assumptions. Completing the actual geometry and parameter identification is a separate validation step.

A separate [derivation verification](../tests/test_nonlinear_derivation.m) evaluates the Cartesian kinetic energy using the rotating inertia tensor, differentiates the Lagrangian numerically and compares it with the accelerations produced by the implemented dynamics. It also checks mechanical energy balance. The check passed at 100 random states with the example parameters. The largest Euler-Lagrange residual was 4.02e-10 N m, and the largest energy-rate residual was 6.29e-10 W.

Run the check from the project folder:

```matlab
addpath('tests')
test_nonlinear_derivation
```
