# Hanging equilibrium analysis

This is the open-loop hanging analysis for Section 4.2. It uses the current torque-input CAD plant, including its 180-degree mounting rotation. The state order is arm angle, pendulum angle, arm rate, and pendulum rate. Angles are in radians and the input is shaft torque in N m.

At zero torque, the pendulum hangs at 180 degrees and both joint rates are zero. The arm can rest at any angle because rotating the arm about its vertical axis does not change gravitational potential. The linearization below uses zero arm angle as the representative equilibrium.

For small deviations from hanging, the state model is `delta_x_dot = A*delta_x + B*tau`. The measured outputs are the two angle deviations, so `C = [eye(2) zeros(2)]` and `D = zeros(2,1)`. The undamped matrices, rounded for display, are:

```text
A = [ 0     0         1   0
      0     0         0   1
      0    42.1493    0   0
      0  -158.2742    0   0 ]

B = [0; 0; 6708.1250; -5779.7595]
```

| Case | Poles (1/s) | Interpretation |
| --- | --- | --- |
| Undamped | 0, 0, ±12.58071i | Small pendulum oscillations persist at about 2.00 Hz. The double zero has one independent eigenvector, so a free-arm drift term can grow in the linearized angle error. The full state does not converge to the selected equilibrium. |
| Demo damping | 0, -0.56893, -0.34492 ± 12.57215i | The pendulum oscillations and joint rates decay locally. The simple zero is the unconstrained arm angle, so trajectories can settle at hanging with a different arm angle. |

The damped case uses the demo's assumed arm and pendulum viscous coefficients of 0.00011 and 0.000024 N m s/rad. The results describe those simulated conditions. For comparison, the undamped upright linearization has a positive real pole of about +12.58 per second.

The MATLAB script checks the zero-torque equilibrium, compares complex-step and central-difference Jacobians, and checks the linear response against a one-second nonlinear integration. The Jacobians agree within 7.74e-9. Across both cases the maximum angle mismatch is 3.15e-7 rad and the maximum rate mismatch is 4.43e-6 rad/s.

Run `analyze_furuta_hanging` from the project root to regenerate the [full matrices](../output/multibody/hanging_state_space.mat) and [pole table](../output/multibody/hanging_equilibrium_poles.csv).
