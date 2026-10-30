# Miura Thesis Simplified

Install Python dependencies with `python -m pip install mujoco`, then run:

```sh
python run_mujoco.py
# The previous command also works:
python run_mujoco.py robot.xml
# Optional alternative configuration:
python run_mujoco.py --config experiment.json
```

Edit `config.json` and restart the viewer to apply changes. The runner applies
settings before compiling the model; `robot.xml` and the meshes stay unchanged.
Launching the XML directly in another viewer bypasses the config.
Missing settings use defaults; unknown keys and invalid values produce errors.

| Setting | Meaning / units |
| --- | --- |
| gravity_m_s2 | Gravity vector [x, y, z], m/s²; [0, 0, 0] disables gravity |
| timestep | Physics step, seconds |
| panel_mass_kg | Mass of each bare panel, kg (0.05 = 50 g); excludes attached hardware |
| motor_mass_kg | Mass of each of 16 motor housings, kg |
| wheel_mass_kg | Mass of each wheel/tire, kg |
| gear_mass_kg | Mass of each gear (three per motor), kg |
| coupler_mass_kg | Mass of each output coupler, kg |
| motor_torque_limit_nm | Symmetric torque command limit per motor, N·m; 0 disables actuation |
| gear_efficiency | Torque multiplier, 0–1; applied shaft torque limit = torque limit × efficiency |
| hinge_damping | Passive fold hinge viscous damping, N·m·s/rad |
| motor_damping | Viscous damping on motor, idler, and output joints, N·m·s/rad |
| wheel_friction | Wheel–ground sliding friction coefficient (dimensionless); applied directly to all 16 tire/floor contact pairs |
| ground_friction | Floor sliding friction coefficient for contacts with non-wheel parts |
| body_friction | Panel and motor housing sliding friction coefficient |
| torsional_friction | Contact torsional friction coefficient, m |
| rolling_friction | Contact rolling friction coefficient, m |
| fixed_base | true anchors panel_0; false lets the whole robot move |
| initial_height_m | Initial panel_0 origin height above the floor, m |

Panel and motor inertia scale proportionally with their mass, preserving their
shape and center of mass. Gear and wheel inertia are recomputed from geometry.
Masses must be positive. All settings use SI units.

For example, change `"wheel_friction": 0.9` to `0.1` for less wheel traction.
The wheel–ground coefficient is assigned directly to explicit contact pairs,
so `ground_friction` cannot override it. MuJoCo computes the contact forces
from the load and relative motion, and those forces act on the wheels and the
whole robot through its joints. No extra artificial push is applied. Available
sliding traction is approximately bounded by coefficient × normal load.
Friction acts only while a wheel is contacting the floor; it is not air drag.

A value of `0` selects frictionless wheel–ground contacts: normal support remains,
but sliding and spin friction are disabled for these pairs. Positive values use
the original four-dimensional wheel contacts, including the separate
`torsional_friction` about the contact normal. `rolling_friction` is not active
for these contacts. Other body/floor contacts
still combine geom coefficients, usually taking the larger value.

This first step adds physics configuration only. The passive viewer starts with
zero motor commands; use its actuator controls to apply torque. Changing motor
strength changes available torque, not an automatic fold target. The existing
1:1 gearing, closure constraints, geometry, and collisions are preserved.
Efficiency is a constant torque approximation, not an electrical motor model or
a full gearbox loss model. There is no motor torque-speed curve or battery model.
The default robot starts 0.2 m above the floor and falls onto it.


## Connected module grids

```sh
python run_mujoco.py -w 3 -h 2
python run_mujoco.py robot.xml -w 2 -h 2 --config config.json
python run_mujoco.py --help
```

`-w`/`--width` specifies columns and `-h`/`--height` specifies rows. Both must
be positive integers and default to 1. `-h` now means height; use `--help` for
help. A 3 × 2 grid contains six complete four-panel robots and 96 motor controls.

Columns extend along world X, rows along world Y. Zero-based row/column names
identify controls, e.g. `r0_c1_p0_m0` is panel 0, motor 0 of the second module
in the first row. A 1 × 1 model keeps the original names. Shared meshes are
loaded once. All modules use the same config for masses, torque, and friction.

The supplied 150 mm panel geometry is tiled at 300 mm column spacing and
259.808 mm row spacing. Two panel edges meet at each module boundary. Each
edge is connected with two point constraints along its crease, producing a
passive hinge with finite solver compliance. Connections transmit forces and
permit folding about their edge; they add no motors or torque-speed model.
These are idealized edge hinges, not a detailed mating-gear/contact model.
The layout assumes the supplied flat four-panel XML and its reference frames;
other robot geometries need different edge locations and spacing.

When `fixed_base` is true, only module `r0_c0` is anchored, and other modules
are free to move subject to their connections. When false, the assembly is
free to move as a whole. Existing intra-module motor couplings and wheel-floor friction are preserved.
For grids, panel collision masks are converted to same-panel body exclusions
so separate modules can collide without losing the original same-panel
filtering. Edge attachments themselves are physical constraints, not collision
forces. Larger grids require more computation and can run slower
than real time, especially during contact or rapid actuation.
