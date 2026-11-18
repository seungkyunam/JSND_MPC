import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('validate_controller', ROOT / 'launch/validate_controller.launch.py')
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


@pytest.mark.parametrize('controller,sim', [('pp', 'false'), ('pp', 'true'), ('mpc', 'true')])
def test_allowed_controller_modes(controller, sim):
    context = LaunchContext()
    context.launch_configurations.update(controller=controller, sim=sim)
    assert VALIDATOR.validate_controller(context) == []


@pytest.mark.parametrize('controller,sim', [('mpc', 'false'), ('mpc', 'True'), ('typo', 'true'), ('', 'true')])
def test_disallowed_modes_fail_before_launch(controller, sim):
    context = LaunchContext()
    context.launch_configurations.update(controller=controller, sim=sim)
    with pytest.raises(ValueError):
        VALIDATOR.validate_controller(context)


def test_validator_precedes_base_and_default_is_pp():
    root = ET.parse(ROOT / 'launch/race.launch.xml').getroot()
    controller_arg = next(item for item in root.findall('arg') if item.get('name') == 'controller')
    assert controller_arg.get('default') == 'pp'
    children = list(root)
    validator = next(index for index, item in enumerate(children)
                     if item.tag == 'include' and 'validate_controller.launch.py' in item.get('file', ''))
    base = next(index for index, item in enumerate(children)
                if item.tag == 'include' and 'base_system.launch.xml' in item.get('file', ''))
    assert validator < base
