"""
Static checks of the Gazebo contact model of the Mecanum forklift (the simulator does not run).

The rollers are not modelled. Each wheel is a sphere with an anisotropic friction. The
friction direction fdir1 must be expressed in the robot frame (base_footprint), not in the
wheel frame: the wheel link rotates, and a direction in the wheel frame would rotate with it.
"""

import pathlib
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
EXTENSION = PKG / 'urdf' / 'forklift_gazebo.xacro'


def description_dir():
    src = PKG.parent / 'forklift_description'
    if src.exists():
        return src
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory('forklift_description'))


OVERLAY = description_dir() / 'config' / 'forklift_mecanum.yaml'
MEC = yaml.safe_load(OVERLAY.read_text())['mecanum']
# X layout: the wheel pushes the ground along these directions (robot frame, x forward, y left).
EXPECTED_DIRECTION = {
    'front_left_drive_wheel': (1, -1, 0),
    'rear_right_drive_wheel': (1, -1, 0),
    'front_right_drive_wheel': (1, 1, 0),
    'rear_left_drive_wheel': (1, 1, 0),
}
gz = shutil.which('gz')


@pytest.fixture(scope='module')
def mecanum_urdf():
    return xacro.process_file(
        str(description_dir() / 'urdf' / 'forklift.urdf.xacro'),
        mappings={'sim_backend': 'gazebo', 'extensions': str(EXTENSION),
                  'controllers_file': '/tmp/controllers.yaml',
                  'params_overlay': str(OVERLAY)}).toxml()


@pytest.fixture(scope='module')
def mecanum_sdf(mecanum_urdf, tmp_path_factory):
    if gz is None:
        pytest.skip('gz CLI not available')
    f = tmp_path_factory.mktemp('sdf') / 'forklift_mecanum.urdf'
    f.write_text(mecanum_urdf)
    out = subprocess.run([gz, 'sdf', '-p', str(f)], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    # The SDF root does not declare the gz: namespace. Remove the prefix for the parser.
    return ET.fromstring(out.stdout.replace('gz:expressed_in', 'expressed_in'))


def wheel_collisions(sdf):
    found = {}
    for col in sdf.iter('collision'):
        name = col.get('name', '')
        for wheel in EXPECTED_DIRECTION:
            if name == f'{wheel}_link_collision':
                found[wheel] = col
    return found


def test_each_wheel_has_exactly_one_collision(mecanum_sdf):
    cols = wheel_collisions(mecanum_sdf)
    assert set(cols) == set(EXPECTED_DIRECTION)
    # The friction extension must merge into the wheel collision. It must not add a second one.
    names = [c.get('name') for c in mecanum_sdf.iter('collision')
             if 'drive_wheel' in c.get('name')]
    assert len(names) == 4
    for col in cols.values():
        assert col.find('geometry/sphere') is not None


@pytest.mark.parametrize('wheel', sorted(EXPECTED_DIRECTION))
def test_wheel_friction_is_anisotropic_in_the_robot_frame(mecanum_sdf, wheel):
    ode = wheel_collisions(mecanum_sdf)[wheel].find('surface/friction/ode')
    assert float(ode.findtext('mu')) == pytest.approx(MEC['mu'])
    assert float(ode.findtext('mu2')) == pytest.approx(MEC['mu2'])
    assert float(ode.findtext('mu')) > float(ode.findtext('mu2')), 'must be anisotropic'
    fdir1 = ode.find('fdir1')
    assert fdir1.get('expressed_in') == 'base_footprint'
    assert tuple(float(v) for v in fdir1.text.split()) == EXPECTED_DIRECTION[wheel]


def test_the_frame_of_fdir1_is_a_link_of_the_model(mecanum_sdf):
    links = {link.get('name') for link in mecanum_sdf.iter('link')}
    assert 'base_footprint' in links


def test_mecanum_robot_has_no_caster_contacts(mecanum_sdf):
    assert not [c for c in mecanum_sdf.iter('collision') if 'caster' in c.get('name', '')]


def test_the_differential_robot_has_no_fdir1():
    urdf = xacro.process_file(
        str(description_dir() / 'urdf' / 'forklift.urdf.xacro'),
        mappings={'sim_backend': 'gazebo', 'extensions': str(EXTENSION),
                  'controllers_file': '/tmp/controllers.yaml'}).toxml()
    assert 'fdir1' not in urdf
