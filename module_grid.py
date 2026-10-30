"""Tile complete four-panel modules and join neighboring panel edges."""
from copy import deepcopy
from itertools import combinations
import math
import xml.etree.ElementTree as ET

# Nominal outer crease geometry of the supplied 150 mm Miura panels.
EDGE = 0.15
RISE = EDGE * math.sqrt(3) / 2
PITCH_X = 2 * EDGE
PITCH_Y = 2 * RISE
REFERENCE_ATTRIBUTES = {
    'body', 'body1', 'body2', 'joint', 'joint1', 'joint2',
    'site', 'site1', 'site2', 'geom', 'geom1', 'geom2',
    'tendon', 'tendon1', 'tendon2', 'cranksite', 'slidersite',
}


def positive_int(value):
    """argparse-compatible positive module count."""
    import argparse
    try:
        number = int(value)
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError('module count must be a positive integer')
    if number < 1:
        raise argparse.ArgumentTypeError('module count must be a positive integer')
    return number


def grid_xml(xml, width=1, height=1):
    """Return an assembled model; one module preserves all original names.

    Two point constraints along each shared edge leave rotation about that
    edge free. These are passive connections, with finite solver compliance.
    Asset meshes are shared, while bodies, controls and constraints are copied.
    """
    if type(width) is not int or type(height) is not int or min(width, height) < 1:
        raise ValueError('width and height must be positive integers')
    if width == height == 1:
        return xml
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    template = world.find("body[@name='panel_0']")
    if template is None:
        raise ValueError('Module grid requires the supplied four-panel robot.xml')
    for i in range(1, 4):
        if template.find(f".//body[@name='panel_{i}']") is None:
            raise ValueError('Module grid requires four named panels')
    fixed = template.find('freejoint') is None
    sections = {}
    for tag in ('equality', 'actuator', 'contact'):
        section = root.find(tag)
        if section is None:
            section = ET.SubElement(root, tag)
        sections[tag] = (section, list(section))
        for child in list(section):
            section.remove(child)
    world.remove(template)
    local_names = {e.get('name') for e in template.iter() if e.get('name')}
    for _, children in sections.values():
        local_names.update(e.get('name') for child in children for e in child.iter() if e.get('name'))

    def renamed(element, prefix):
        clone = deepcopy(element)
        for e in clone.iter():
            if 'name' in e.attrib:
                e.set('name', prefix + e.get('name'))
            for attr in REFERENCE_ATTRIBUTES & e.attrib.keys():
                if e.get(attr) in local_names:
                    e.set(attr, prefix + e.get(attr))
        return clone

    modules = {}
    for row in range(height):
        for col in range(width):
            prefix = f'r{row}_c{col}_'
            body = renamed(template, prefix)
            pos = [float(v) for v in body.get('pos', '0 0 0').split()]
            pos[0] += (col - (width - 1) / 2) * PITCH_X
            pos[1] += (row - (height - 1) / 2) * PITCH_Y
            body.set('pos', ' '.join(map(str, pos)))
            # fixed_base anchors only the first module, not every module.
            if fixed and (row or col):
                body.insert(0, ET.Element('freejoint', name=prefix + 'base'))
            world.append(body)
            modules[row, col] = (prefix, body)
            for section, children in sections.values():
                for child in children:
                    section.append(renamed(child, prefix))
            # Single-module masks encode panel identity in four bits. Reusing
            # those bits across copies would suppress some inter-module
            # collisions. Use one robot bit and explicit same-panel exclusions.
            groups = {}

            def collision_groups(current, owner=None):
                if current.get('name', '').startswith(prefix + 'panel_'):
                    owner = current.get('name')
                for geom in current.findall('geom'):
                    if int(geom.get('contype', '1')) > 0:
                        groups.setdefault(owner, set()).add(current.get('name'))
                        affinity = int(geom.get('conaffinity', '1'))
                        geom.set('contype', '2')
                        geom.set('conaffinity', str(2 | (affinity & 1)))
                for child in current.findall('body'):
                    collision_groups(child, owner)

            collision_groups(body)
            for group in groups.values():
                for first, second in combinations(sorted(group), 2):
                    ET.SubElement(sections['contact'][0], 'exclude',
                                  body1=first, body2=second)

    equality = sections['equality'][0]

    def panel_body(module, index):
        prefix, body = modules[module]
        return body if index == 0 else body.find(f".//body[@name='{prefix}panel_{index}']")

    def hinge(label, first, panel_a, second, panel_b, start, end, shift):
        # All original panel reference frames coincide in the flat pose.
        # Therefore seam coordinates are already panel-local coordinates.
        for n, fraction in enumerate((0.25, 0.75)):
            point = [a + fraction * (b - a) for a, b in zip(start, end)]
            other = [p - delta for p, delta in zip(point, shift)]
            site_a, site_b = label + f'_{n}_a', label + f'_{n}_b'
            for body, name, position in ((panel_body(first, panel_a), site_a, point),
                                         (panel_body(second, panel_b), site_b, other)):
                ET.SubElement(body, 'site', name=name, pos=' '.join(map(str, position)),
                              size='.0015', rgba='1 .6 .1 1', group='4')
            ET.SubElement(equality, 'connect', name=label + f'_{n}',
                          site1=site_a, site2=site_b, solref='.005 1')

    for row in range(height):
        for col in range(width):
            here = (row, col)
            if col + 1 < width:
                right = (row, col + 1)
                for side, a, b, sign in (('upper', 0, 1, 1), ('lower', 3, 2, -1)):
                    hinge(f'seam_x_r{row}_c{col}_{side}', here, a, right, b,
                          (EDGE, 0, 0), (1.5 * EDGE, sign * RISE, 0), (PITCH_X, 0, 0))
            if row + 1 < height:
                above = (row + 1, col)
                for side, a, b, x in (('right', 0, 3, EDGE / 2), ('left', 1, 2, -EDGE / 2)):
                    hinge(f'seam_y_r{row}_c{col}_{side}', here, a, above, b,
                          (x, RISE, 0), (x + EDGE, RISE, 0), (0, PITCH_Y, 0))
    floor = world.find("geom[@name='floor']")
    if floor is not None:
        floor.set('size', f'{max(2, width * PITCH_X)} {max(2, height * PITCH_Y)} .1')
    root.set('model', f'miura_{width}x{height}')
    return ET.tostring(root, encoding='unicode')
