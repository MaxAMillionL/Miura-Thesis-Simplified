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
| wheel_friction | Tire sliding friction coefficient |
| ground_friction | Floor sliding friction coefficient |
| body_friction | Panel and motor housing sliding friction coefficient |
| torsional_friction | Contact torsional friction coefficient, m |
| rolling_friction | Contact rolling friction coefficient, m |
| fixed_base | true anchors panel_0; false lets the whole robot move |
| initial_height_m | Initial panel_0 origin height above the floor, m |

Panel and motor inertia scale proportionally with their mass, preserving their
shape and center of mass. Gear and wheel inertia are recomputed from geometry.
Masses must be positive. All settings use SI units. MuJoCo combines friction
parameters for each contact (usually the larger coefficient), so adjust both
floor and contacting body coefficients when testing a slippery surface.

This first step adds physics configuration only. The passive viewer starts with
zero motor commands; use its actuator controls to apply torque. Changing motor
strength changes available torque, not an automatic fold target. The existing
1:1 gearing, closure constraints, geometry, and collisions are preserved.
Efficiency is a constant torque approximation, not an electrical motor model or
a full gearbox loss model. There is no motor torque-speed curve or battery model.
The default robot starts 0.2 m above the floor and falls onto it.
