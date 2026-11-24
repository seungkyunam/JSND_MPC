"""Only temporary-directory deployment/rollback tests; no Jetson connection."""
import importlib.util
import json
from pathlib import Path
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('step45_deploy', ROOT/'jsnd_int/run/step45_deploy.py')
deploy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(deploy)


def baseline(target):
    manifest = json.loads(deploy.MANIFEST.read_text())
    for name, hashes in manifest['files'].items():
        if hashes['base_sha256'] is not None:
            path = target/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(subprocess.check_output(['git', 'show',
                f"{manifest['base_commit']}:jsnd_int/racing_stack/{name}"], cwd=ROOT))
    return manifest


def test_apply_restore_preserves_physical_config_and_estop(tmp_path):
    target, backup = tmp_path/'target', tmp_path/'backup'
    manifest = baseline(target)
    sentinel = target/'stack_master/config/mpc_car.yaml'
    sentinel.parent.mkdir(parents=True)
    sentinel.write_text('local physical settings remain untouched')
    deploy.apply(target, backup)
    for name, hashes in manifest['files'].items():
        assert deploy.digest(target/name) == hashes['new_sha256']
    deploy.restore(backup)
    for name, hashes in manifest['files'].items():
        assert deploy.digest(target/name) == hashes['base_sha256']
    assert sentinel.read_text() == 'local physical settings remain untouched'


def test_unknown_target_is_rejected_before_any_copy(tmp_path):
    target = tmp_path/'target'
    manifest = baseline(target)
    name = next(iter(manifest['files']))
    (target/name).write_text('Jetson local change')
    with pytest.raises(RuntimeError, match='destination differs'):
        deploy.apply(target, tmp_path/'backup')
    assert not (tmp_path/'backup').exists()
    assert (target/name).read_text() == 'Jetson local change'


def test_restore_refuses_to_destroy_post_install_changes(tmp_path):
    target, backup = tmp_path/'target', tmp_path/'backup'
    manifest = baseline(target)
    deploy.apply(target, backup)
    name = next(iter(manifest['files']))
    (target/name).write_text('new local work')
    with pytest.raises(RuntimeError, match='changed after deployment'):
        deploy.restore(backup)
    assert (target/name).read_text() == 'new local work'
