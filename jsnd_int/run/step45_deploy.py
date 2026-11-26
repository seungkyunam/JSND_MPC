#!/usr/bin/env python3
"""Local-only manifest-based overlay install/rollback. Never connects to a vehicle.

Refuses unknown destination revisions; copies only files in the selected manifest.
The default is the historical Step 45 manifest; Step 51 supplies --manifest.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[2]
OVERLAY = ROOT / 'jsnd_int/racing_stack'
MANIFEST = Path(__file__).with_name('step45_deploy_manifest.json')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else None


def check(target):
    manifest = json.loads(MANIFEST.read_text())
    for name, hashes in manifest['files'].items():
        if digest(OVERLAY / name) != hashes['new_sha256']:
            raise RuntimeError(f'checkout differs from release manifest: {name}')
        if digest(target / name) != hashes['base_sha256']:
            raise RuntimeError(f'destination differs from manifest baseline: {name}; review local changes, do not force-copy')
    return manifest


def apply(target, backup):
    manifest = check(target)
    backup.mkdir(parents=True, exist_ok=False)
    record = dict(target=str(target), files=manifest['files'])
    for name, hashes in manifest['files'].items():
        if hashes['base_sha256'] is not None:
            dst = backup / 'original' / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target / name, dst)
    (backup / 'restore.json').write_text(json.dumps(record, indent=2) + '\n')
    for name in manifest['files']:
        dst = target / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(OVERLAY / name, dst)
    print(f'Applied {len(manifest["files"])} files. Rollback backup: {backup}')


def restore(backup):
    record = json.loads((backup / 'restore.json').read_text())
    target = Path(record['target'])
    # Preflight ALL files before changing any; also permits interrupted installs.
    for name, hashes in record['files'].items():
        if digest(target / name) not in (hashes['base_sha256'], hashes['new_sha256']):
            raise RuntimeError(f'destination changed after deployment: {name}; preserve/review those changes first')
        if digest(backup / 'original' / name) != hashes['base_sha256']:
            raise RuntimeError(f'backup corrupted: {name}')
    for name, hashes in record['files'].items():
        if hashes['base_sha256'] is None:
            (target / name).unlink(missing_ok=True)
        else:
            shutil.copy2(backup / 'original' / name, target / name)
    print(f'Restored source in {target}; rebuild the affected ROS packages before launch.')


def main():
    global MANIFEST
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['check', 'apply', 'restore'])
    parser.add_argument('--target', type=Path, default=Path.home() / 'JSND_INT/src/racing_stack')
    parser.add_argument('--backup', type=Path)
    parser.add_argument('--manifest', type=Path, default=MANIFEST)
    args = parser.parse_args()
    MANIFEST = args.manifest
    if args.action != 'check' and args.backup is None:
        parser.error('--backup is required for apply/restore')
    if args.action == 'check':
        check(args.target.resolve())
        print('All destination files match the selected manifest baseline.')
    elif args.action == 'apply':
        apply(args.target.resolve(), args.backup.resolve())
    else:
        restore(args.backup.resolve())


if __name__ == '__main__':
    main()
