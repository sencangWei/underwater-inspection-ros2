#!/usr/bin/env python3
"""Build our inspection scene, preserving the upstream BlueROV2 model."""
from pathlib import Path
from xml.etree import ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'src/underwater_inspection'
DEST = PKG / 'scenarios'
DEST.mkdir(exist_ok=True)
cfg = yaml.safe_load((PKG / 'config/mission.yaml').read_text())

# Copied from the locked GPL upstream; the generated model retains its geometry.
model = ET.parse(ROOT / 'third_party/stonefish_sim/stonefish_description/data/robots/bluerov2/bluerov2.scn')
robot = model.getroot().find('robot')
for name in ('camera_left', 'camera_right', 'fls'):
    sensor = robot.find(f"sensor[@name='{name}']")
    sensor.set('rate', '10.0')
    sensor.find('origin').set('rpy', '1.57079632679 0.0 1.57079632679')
    if name == 'fls':
        sensor.find('specs').set('beams', '128')
        sensor.find('specs').set('bins', '256')
        sensor.find('settings').set('range_max', '20.0')
for name, typ, rate in [('imu', 'imu', '50.0'), ('pressure', 'pressure', '10.0')]:
    sensor = ET.SubElement(robot, 'sensor', name=name, type=typ, rate=rate)
    ET.SubElement(sensor, 'link', name='base_link')
    ET.SubElement(sensor, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(sensor, 'ros_publisher', topic=f'/bluerov2/{name}')
    ET.SubElement(sensor, 'noise', **({'angle': '0.002', 'angular_velocity': '0.001',
                                      'linear_acceleration': '0.005'} if typ == 'imu'
                                     else {'pressure': '20.0'}))
ET.indent(model, space='  ')
model.write(DEST / 'robot.scn', encoding='unicode', xml_declaration=True)

root = ET.Element('scenario')
environment = ET.SubElement(root, 'environment')
ET.SubElement(environment, 'ned', latitude='41.7777', longitude='3.0333')
ocean = ET.SubElement(environment, 'ocean')
ET.SubElement(ocean, 'water', density='1031.0', jerlov='0.1', temperature='15.0')
ET.SubElement(ocean, 'particles', enabled='true')
ET.SubElement(ocean, 'waves', height='0.0')
current = ET.SubElement(ocean, 'current', type='uniform')
ET.SubElement(current, 'velocity', xyz=' '.join(map(str, cfg['water_current'])))
atmosphere = ET.SubElement(environment, 'atmosphere')
ET.SubElement(atmosphere, 'sun', azimuth='40.0', elevation='65.0')
ET.SubElement(root, 'include', file='data/worlds/common/visual.scn')
materials = ET.parse(ROOT / 'third_party/stonefish_sim/stonefish_description/data/worlds/common/materials.scn').getroot().find('materials')
root.append(materials)
ET.SubElement(materials, 'material', name='Steel', density='7850.0', restitution='0.1')
looks = ET.SubElement(root, 'looks')
ET.SubElement(looks, 'look', name='inspection_yellow', rgb='1.0 0.86 0.01', roughness='0.6')
ET.SubElement(looks, 'look', name='inspection_red', rgb='0.9 0.03 0.02', roughness='0.4')
ET.SubElement(looks, 'look', name='crate_brown', rgb='0.55 0.30 0.12', roughness='0.85')
ET.SubElement(looks, 'look', name='dock_cyan', rgb='0.0 0.85 0.90', roughness='0.4')


def static(name, typ, xyz, look, dims, rpy='0 0 0'):
    item = ET.SubElement(root, 'static', name=name, type=typ)
    ET.SubElement(item, 'dimensions', **dims)
    ET.SubElement(item, 'material', name='Concrete')
    ET.SubElement(item, 'look', name=look)
    ET.SubElement(item, 'world_transform', xyz=' '.join(map(str, xyz)), rpy=rpy)


static('seafloor', 'box', (0, 0, 6.15), 'dark_sand', {'xyz': '30 18 0.3'})
static('home_pad', 'box', (-5, 0, 5.97), 'dock_cyan', {'xyz': '1.5 1.5 0.05'})
for obstacle in cfg['obstacles']:
    name, x, y, radius = (obstacle[k] for k in ('name', 'x', 'y', 'radius'))
    if name.startswith('pillar'):
        static(name, 'cylinder', (x, y, 3.3), 'gray',
               {'radius': str(radius), 'height': '5.2'})
    else:
        static(name, 'box', (x, y, 3.9), 'crate_brown', {'xyz': '1.1 1.1 4.0'})
static('pipeline', 'cylinder', (0, 3.6, 5.3), 'gray',
       {'radius': '0.22', 'height': '11.0'}, rpy='0 1.57079632679 0')
x, y, z = cfg['target']
static('panel_frame', 'box', (x + .12, y, z), 'gray', {'xyz': '0.2 1.5 1.3'})
static('inspection_panel', 'box', (x, y, z), 'inspection_yellow', {'xyz': '0.12 1.2 1.0'})
static('valve', 'cylinder', (x - .12, y, z), 'inspection_red',
       {'radius': '0.18', 'height': '0.15'}, rpy='0 1.57079632679 0')
include = ET.SubElement(root, 'include', file='$(find underwater_inspection)/scenarios/robot.scn')
for name, val in [('vehicle_name', 'bluerov2'), ('xyz', ' '.join(map(str, cfg['spawn']))),
                  ('rpy', '0 0 0')]:
    ET.SubElement(include, 'arg', name=name, value=val)
for body in ['seafloor', 'inspection_panel', 'panel_frame'] + [o['name'] for o in cfg['obstacles']]:
    contact = ET.SubElement(root, 'contact', name='collision_' + body)
    ET.SubElement(contact, 'first_body', name='bluerov2/base_link')
    ET.SubElement(contact, 'second_body', name=body)
    ET.SubElement(contact, 'history', points='1')
    ET.SubElement(contact, 'ros_publisher', topic='/inspection/contact/' + body)
ET.indent(root, space='  ')
ET.ElementTree(root).write(DEST / 'inspection.scn', encoding='unicode', xml_declaration=True)
print(f'Generated {DEST}; forward stereo camera and FLS, {len(cfg["obstacles"])} navigation obstacles.')
