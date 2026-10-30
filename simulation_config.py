"""Apply editable SI-unit physics settings before MuJoCo compiles the model."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

DEFAULTS = {
    'gravity_m_s2': [0, 0, -9.81], 'timestep': 0.001,
    'panel_mass_kg': 0.05, 'motor_mass_kg': 0.01,
    'wheel_mass_kg': 0.003, 'gear_mass_kg': 0.002,
    'coupler_mass_kg': 0.0003, 'motor_torque_limit_nm': 0.05,
    'gear_efficiency': 0.8, 'hinge_damping': 0.005,
    'motor_damping': 0.000002, 'wheel_friction': 0.9,
    'ground_friction': 0.9, 'body_friction': 0.9,
    'torsional_friction': 0.002, 'rolling_friction': 0.0001,
    'fixed_base': False, 'initial_height_m': 0.2,
}


def load_config(path):
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(config, dict):
        raise ValueError('Config must be a JSON object')
    unknown = config.keys() - DEFAULTS.keys()
    if unknown:
        raise ValueError(f'Unknown config settings: {sorted(unknown)}')
    config = DEFAULTS | config
    for key, value in config.items():
        if key == 'fixed_base':
            if type(value) is not bool:
                raise ValueError('fixed_base must be true or false')
            continue
        values = value if key == 'gravity_m_s2' else [value]
        if key == 'gravity_m_s2' and (not isinstance(values, list) or len(values) != 3):
            raise ValueError('gravity_m_s2 must contain three numbers')
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
            raise ValueError(f'{key} must contain finite numbers')
        if key == 'gravity_m_s2':
            continue
        if value < 0 or ((key.endswith('_mass_kg') or key == 'timestep') and value == 0):
            raise ValueError(f'{key} has an invalid value')
        if key == 'gear_efficiency' and value > 1:
            raise ValueError('gear_efficiency must be between 0 and 1')
    return config


def configured_xml(xml_path, config):
    """Return configured XML; leave the source XML unchanged."""
    xml_path = Path(xml_path).resolve()
    root = ET.parse(xml_path).getroot()
    compiler = root.find('compiler')
    for attribute in ('meshdir', 'texturedir'):
        compiler.set(attribute, str((xml_path.parent / compiler.get(attribute, '.')).resolve()))
    option = root.find('option')
    option.set('gravity', ' '.join(map(str, config['gravity_m_s2'])))
    option.set('timestep', str(config['timestep']))
    base = root.find(".//body[@name='panel_0']")
    pos = base.get('pos').split()
    pos[2] = str(config['initial_height_m'])
    base.set('pos', ' '.join(pos))
    if config['fixed_base']:
        base.remove(base.find('freejoint'))
    for body in root.iter('body'):
        inertial = body.find('inertial')
        if inertial is None:
            continue
        key = 'panel_mass_kg' if body.get('name', '').startswith('panel_') else 'motor_mass_kg'
        ratio = config[key] / float(inertial.get('mass'))
        inertial.set('mass', str(config[key]))
        for attribute in ('fullinertia', 'diaginertia'):
            if attribute in inertial.attrib:
                inertial.set(attribute, ' '.join(str(float(v) * ratio) for v in inertial.get(attribute).split()))
    for geom in root.iter('geom'):
        name = geom.get('name', '')
        for suffix, key in (('_gear_visual', 'gear_mass_kg'), ('_tire', 'wheel_mass_kg'), ('_coupler', 'coupler_mass_kg')):
            if name.endswith(suffix):
                geom.set('mass', str(config[key]))
        if name == 'floor' or name.endswith(('_tire', '_collision')) or name.startswith('panel_ground_'):
            key = 'ground_friction' if name == 'floor' else ('wheel_friction' if name.endswith('_tire') else 'body_friction')
            geom.set('friction', f"{config[key]} {config['torsional_friction']} {config['rolling_friction']}")
    # Explicit pairs make wheel_friction the wheel/floor coefficient instead
    # of allowing the floor's coefficient to override it through geom mixing.
    # MuJoCo resolves these contacts and transmits their forces through the
    # wheel joints to the robot; no separate body force should be added.
    contact = root.find('contact')
    if contact is None:
        contact = ET.SubElement(root, 'contact')
    mu = config['wheel_friction']
    spin = config['torsional_friction']
    roll = config['rolling_friction']
    for tire in root.findall('.//geom'):
        name = tire.get('name', '')
        if not name.endswith('_tire'):
            continue
        pair = next((p for p in contact.findall('pair')
                     if {p.get('geom1'), p.get('geom2')} == {'floor', name}), None)
        if pair is None:
            pair = ET.SubElement(contact, 'pair', geom1='floor', geom2=name)
        # Preserve the original four-dimensional contact: normal force,
        # two sliding friction forces, and spin friction torque.
        # Use truly frictionless normal-only contacts at zero grip rather
        # than a friction cone with zero-width axes.
        pair.set('condim', '1' if mu == 0 else '4')
        pair.set('friction', f'{mu} {mu} {spin} {roll} {roll}')
    for joint in root.findall('.//body/joint'):
        key = 'hinge_damping' if joint.get('name', '').startswith('fold_') else 'motor_damping'
        joint.set('damping', str(config[key]))
    for motor in root.findall('./actuator/motor'):
        limit = config['motor_torque_limit_nm']
        motor.set('ctrlrange', f'{-limit} {limit}')
        motor.set('ctrllimited', 'true')
        motor.set('forcerange', f'{-limit} {limit}')
        motor.set('forcelimited', 'true')
        motor.set('gear', str(config['gear_efficiency']))
    return ET.tostring(root, encoding='unicode')


def load_model(xml_path, config_path, *, width=1, length=1):
    import mujoco
    from module_grid import grid_xml
    xml = configured_xml(xml_path, load_config(config_path))
    return mujoco.MjModel.from_xml_string(grid_xml(xml, width, length))
