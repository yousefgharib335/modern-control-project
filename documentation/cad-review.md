# Inventor CAD review

The current main assembly is [Assembly1.iam](../cad/originals/modern%20control/Assembly1.iam). The base revision adds two grounded occurrences of the same 110 mm aluminum profile and mates them to the motor holder. The revised assembly also changes the poses of the holder and moving assemblies. [Revision extraction and comparison](../output/cad/inventor-base-revision.json).

The section below documents the earlier review of the [pre-base assembly](../cad/originals/modern%20control/Assembly1.pre-base.iam). The source part masses used for the existing mechanism are shared by both revisions. See the [current Simscape plant](multibody-plant.md) for composite inertias after placement changes.

## Extracted mass properties

Masses follow the assigned CAD materials. The table lists unique documents. The main assembly contains two occurrences of the same rotary encoder document.

| Document | Assigned material | Mass (g) |
| --- | --- | --- |
| encoder case.ipt | PC/ABS Plastic | 12.761957 |
| 4mm 5mm 6mm Stainless Steel Hex Shaft Coupling.ipt | Copper | 12.898802 |
| ROTERY ENCODER.step.ipt | 1023 Carbon Steel Sheet (SS) | 362.581649 |
| ALLEN_04_mm.ipt | Steel, Alloy | 15.056582 |
| motor holder.ipt | Aluminum 6061 | 7.523399 |
| Rigid 6 to 8mm Coupler v2.ipt | Copper | 35.709566 |
| Motor.ipt | Steel, Alloy | 117.533764 |

## Pendulum measurements

The Allen-key part mass is **15.056582 g**. Its center of mass in the part coordinate system is approximately **(49.442757, -2.235844, 0.000000) mm**.

Mate:10 identifies the short-leg hinge axis. Transforming the part center of mass into the assembly and projecting perpendicular to this axis gives **lc1 = 49.442757 mm**. This replaces the earlier 45.37 mm subtraction estimate for this supplied part.

The part bounding envelope is 119.90 mm along local X, 28.25 mm along Y and approximately 4.619 mm along Z. A reported full length l1 needs specified endpoints or a centerline convention for this bent shape.

| COM inertia API component | kg m^2 |
| --- | --- |
| Ixx | 5.416130988e-07 |
| Iyy | 2.172465514e-05 |
| Izz | 2.223324089e-05 |
| Ixy | -1.664202282e-06 |
| Iyz | 3.862862165e-18 |
| Ixz | 1.004455864e-17 |

These are the raw Inventor XYZMomentsOfInertia components at the center of mass, in the part reference axes. Their axis orientation and component convention must be retained when constructing the Simscape inertia tensor. [Inventor inertia API](https://help.autodesk.com/cloudhelp/2023/ENU/Inventor-API/files/MassProperties_XYZMomentsOfInertia.htm)

The pendulum mass and inertia above cover the Allen-key part. Define which coupler and encoder-rotor components swing with it before computing the composite pendulum properties.

## Arm geometry and assembly readiness

The horizontal radius from the driven motor axis to the Allen part's local origin is **69.250 mm**. The center-of-mass plane lies **67.014 mm** along the horizontal hinge direction from the motor axis.

The encoder/pendulum hinge axis points radially, so its infinite line intersects the driven motor axis. The shortest distance between those two infinite axes would be zero. Analytical L0 needs a finite hinge-reference location and a consistent treatment of the bent part COM offset along the hinge. The 69.250 mm geometric offset is therefore labelled by its endpoints.

Two constraints in Assembly2 have health code 11780, which Inventor identifies as DriverLost. Their geometry must be repaired in a working assembly before relying on constraint-based multibody export. Main-assembly constraints report 11778, UpToDate. [Inventor health definitions](https://help.autodesk.com/cloudhelp/2020/ENU/Inventor-API/files/HealthStatusEnum.htm)

The occurrence ROTERY ENCODER.step:1 is ungrounded and has no main-assembly constraints connecting it. Its position is separate from the motor-supported group. Confirm its intended mounting or whether it is a reference occurrence when defining the multibody structure.

The motor and motor holder are grounded. The other encoder, couplers, subassembly and pendulum require explicit assignment to fixed, arm-moving and pendulum-moving groups. This determines the composite arm inertia J0 and pendulum inertia.

## Data for the next modelling step

The [pre-base extraction](../output/cad/inventor-review.json) contains source hashes, local mass properties, occurrence transforms, cylindrical geometry, assembly constraints and the earlier package inventory. Internal CAD lengths are cm and inertias are kg cm^2. The pendulum_part_si section supplies SI conversions.

The Simscape model defines the two revolute joints and groups the subassembly rigidly at its supplied pose. Repair the unresolved mates before attempting constraint-based export. Motor electrical constants, rotor and gearbox inertia, gear ratio, losses and friction still require identification for the physical prototype.

The analytical code still uses its earlier l1 name for pivot-to-COM distance. Rename that quantity to lc1 and define the full length separately when incorporating this CAD model.
