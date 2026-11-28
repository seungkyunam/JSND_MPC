#!/usr/bin/env python3
"""Stage/restore the c16bced final vehicle files into a clean JSND_INT checkout.

This tool is local-only: it never launches ROS or publishes commands. Apply is
refused for a dirty target. Restore is refused if staged files changed afterward.
"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'jsnd_int/racing_stack'
FILES = (
    'controller/controller/combined/src/Controller.py',
    'controller/controller/controller_manager.py',
    'controller/controller/mpc/__init__.py',
    'controller/controller/mpc/bicycle.py',
    'controller/controller/mpc/configuration.py',
    'controller/controller/mpc/integrated_mpc.py',
    'planner/spliner/spliner/static_avoidance_node.py',
    'state_machine/state_machine/state_machine_node.py',
    'state_machine/state_machine/state_machine_params.py',
    'state_machine/state_machine/state_transitions.py',
    'stack_master/launch/race.launch.xml',
    'stack_master/launch/validate_final_vehicle.launch.py',
    'stack_master/config/mpc_car.yaml',
    'stack_master/config/mpc_profile_obstacle_1s.yaml',
    'stack_master/config/mpc_profile_6p0.yaml',
    'stack_master/config/mpc_profile_7p0.yaml',
    'stack_master/config/mpc_profile_8p0.yaml',
    'stack_master/config/mpc_profile_9p0.yaml',
    'stack_master/config/mpc_profile_10p0.yaml',
)
NEW_FILES_ALLOWED = {
    'stack_master/launch/validate_final_vehicle.launch.py',
    'stack_master/config/mpc_profile_obstacle_1s.yaml',
    'stack_master/config/mpc_profile_6p0.yaml',
    'stack_master/config/mpc_profile_7p0.yaml',
    'stack_master/config/mpc_profile_8p0.yaml',
    'stack_master/config/mpc_profile_9p0.yaml',
    'stack_master/config/mpc_profile_10p0.yaml',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def require_clean(target):
    result = subprocess.run(['git', '-C', str(target), 'status', '--porcelain'],
                            check=True, text=True, stdout=subprocess.PIPE)
    if result.stdout.strip():
        raise RuntimeError('target checkout is dirty; preserve it and use a dedicated clean checkout')
    for name in FILES:
        if not (SOURCE / name).is_file():
            raise RuntimeError(f'release source missing: {name}')
        if name not in NEW_FILES_ALLOWED and not (target / name).is_file():
            raise RuntimeError(f'target is not a compatible full racing_stack checkout: {name}')


def apply(target, backup):
    require_clean(target)
    backup.mkdir(parents=True, exist_ok=False)
    record = {'target': str(target), 'release_parent': 'c16bced', 'files': {}}
    for name in FILES:
        destination = target / name
        old_hash = digest(destination)
        if old_hash is not None:
            saved = backup / 'original' / name
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(destination, saved)
        record['files'][name] = {'before': old_hash, 'after': digest(SOURCE / name)}
    (backup / 'restore.json').write_text(json.dumps(record, indent=2) + '\n')
    for name in FILES:
        destination = target / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(SOURCE / name, destination)
    print(f'Staged {len(FILES)} files into {target}. Backup: {backup}')
    print('Build controller, spliner, state_machine and stack_master before launch.')


def restore(backup):
    record = json.loads((backup / 'restore.json').read_text())
    target = Path(record['target'])
    for name, hashes in record['files'].items():
        if digest(target / name) != hashes['after']:
            raise RuntimeError(f'post-stage change detected, refusing restore: {name}')
        if hashes['before'] is not None and digest(backup / 'original' / name) != hashes['before']:
            raise RuntimeError(f'backup checksum mismatch: {name}')
    for name, hashes in record['files'].items():
        destination = target / name
        if hashes['before'] is None:
            destination.unlink(missing_ok=True)
        else:
            shutil.copy2(backup / 'original' / name, destination)
    print(f'Restored {target}; rebuild the same four ROS packages before use.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('check', 'apply', 'restore'))
    parser.add_argument('--target', type=Path)
    parser.add_argument('--backup', type=Path)
    args = parser.parse_args()
    if args.action == 'restore':
        if args.backup is None:
            parser.error('restore requires --backup')
        restore(args.backup.resolve())
        return
    if args.target is None:
        parser.error('check/apply requires --target')
    target = args.target.resolve()
    if args.action == 'check':
        require_clean(target)
        print(f'Compatible clean target: {target}; {len(FILES)} files ready to stage')
    else:
        if args.backup is None:
            parser.error('apply requires --backup')
        apply(target, args.backup.resolve())


if __name__ == '__main__':
    main()
