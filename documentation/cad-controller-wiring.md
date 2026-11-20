# CAD plant and controller wiring

The four controller models each contain the **System** subsystem from [furuta_multibody_electrical.slx](../models/furuta_multibody_electrical.slx). **System** contains the switching **H-Bridge**, **DC Motor**, and **Pendulum Plant**. Each model runs separately with local joint initial conditions and feeds the plant's angles and angular rates back to its controller.

| Controller | CAD model | Feedback from the plant |
| --- | --- | --- |
| PID/PD | [furuta_pid_model.slx](../models/furuta_pid_model.slx) | Arm and pendulum angles. The controller retains its derivative filters. |
| Pole placement | [furuta_pole_placement_model.slx](../models/furuta_pole_placement_model.slx) | Two angles and two angular rates |
| LQR | [furuta_lqr_model.slx](../models/furuta_lqr_model.slx) | Two angles and two angular rates |
| Stateflow and swing-up | [furuta_hybrid_model.slx](../models/furuta_hybrid_model.slx) | Sampled angles and rates. The supervisor retains its PID, pole placement, and LQR selector. |

The standalone controller models start upright at 0° with both joints at rest. They apply a 0.0004 N·m pendulum push from 0.500 to 0.550 s. The standard Phase 2 push verification starts upright. The hybrid model starts hanging at 180° and contains the swing-up stage. Change `initialState`, `pushTorque`, `pushStart`, or `pushDuration` in the corresponding model workspace to configure an interactive run. The inputs are an optional **Test Voltage V** added to the controller command, and **Brake Command**, which overrides drive voltage. The outputs include the states, controller voltage, applied drive torque, winding current, and motor terminal voltage. The standalone models also expose **Push Torque Nm**. The hybrid model outputs the Stateflow mode and suppresses both the controller and test voltage in Safe-Stop. Voltage removal coasts when Brake Command is zero.

All three standalone balance models include **Small Command Compensation** and log the actual command to the bridge as **Bridge Request V**. Their **Controller Voltage V** output is the equivalent voltage request before compensation. Their gains come from the compensated average model. The hybrid model uses the earlier checkout gains and swing-up settings. The [electrical stability analysis](electrical-stability.md) records the averaged current fit and the three balance designs for the switching System.

Open **Pole Placement Controller** or **LQR Controller** to see the standard Simulink **State Feedback -K** matrix Gain and **Voltage Limit** Saturation blocks. The gain and limits are stored in `stateFeedbackDesign` in the corresponding model workspace.

From the project folder in MATLAB, run the following to verify the saved models:

```matlab
verify_furuta_cad_controllers
```

The verifier checks initial conditions, the drive and brake input connections, switching voltage, current-to-torque coupling, and hybrid Safe-Stop. To open a saved model by name, run `addpath('models')` first. After revising the standalone electrical System, run `build_furuta_cad_controllers` to regenerate the controller models with the revised System. The earlier example simulations can be regenerated from their individual builder scripts.

The models expose the simulation signals through their output ports. Click **Run** in Simulink to open Multibody Explorer automatically, then click **Play** in the viewer to replay the run.
