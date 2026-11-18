#!/usr/bin/env python3
"""Compare package trees without writing to the inspected workspace."""

import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGES = {'controller', 'stack_master'}
IGNORED = {'.git', '__pycache__', '.pytest_cache', 'build', 'install', 'log'}


def scan(root):
    root = Path(root)
    if not root.is_dir():
        raise ValueError(f'not a directory: {root}')
    found = {}
    visited = set()
    for directory, subdirs, files in os.walk(root, followlinks=True):
        real = os.path.realpath(directory)
        if real in visited:
            subdirs[:] = []
            continue
        visited.add(real)
        subdirs[:] = sorted(name for name in subdirs if name not in IGNORED and not name.startswith('.'))
        if 'package.xml' not in files:
            continue
        try:
            name = ET.parse(Path(directory) / 'package.xml').getroot().findtext('name')
        except ET.ParseError as exc:
            raise ValueError(f'invalid package.xml in {directory}: {exc}') from exc
        if name in PACKAGES:
            if name in found:
                raise ValueError(f'multiple source packages named {name}: {found[name]} and {directory}')
            found[name] = directory
        subdirs[:] = []
    result = {'packages': found, 'files': {}}
    for name, package_root in sorted(found.items()):
        for directory, subdirs, files in os.walk(package_root, followlinks=False):
            subdirs[:] = sorted(item for item in subdirs if item not in IGNORED and not item.endswith('.egg-info'))
            for item in subdirs:
                if (Path(directory) / item).is_symlink():
                    raise ValueError(f'symlink directory must be reviewed: {Path(directory) / item}')
            for filename in sorted(files):
                if filename.endswith(('.pyc', '.pyo')):
                    continue
                path = Path(directory) / filename
                if path.is_symlink():
                    raise ValueError(f'symlink file must be reviewed: {path}')
                digest = hashlib.sha256()
                with path.open('rb') as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                        digest.update(chunk)
                relative = Path(name) / path.relative_to(package_root)
                result['files'][str(relative)] = digest.hexdigest()
    return result


def compare(local, remote):
    if set(local['packages']) != PACKAGES:
        raise ValueError('local source must contain controller and stack_master')
    print('Existing source packages:')
    for name in sorted(PACKAGES):
        print(f"  {name}: {remote['packages'].get(name, 'NOT FOUND')}")
    local_files = local['files']
    remote_files = remote['files']
    for label, paths in (
        ('DIFFERENT (overlay shadows existing)', sorted(k for k in local_files.keys() & remote_files.keys() if local_files[k] != remote_files[k])),
        ('ONLY IN JSND SNAPSHOT', sorted(local_files.keys() - remote_files.keys())),
        ('ONLY IN VEHICLE SOURCE', sorted(remote_files.keys() - local_files.keys())),
    ):
        print(f'\n{label}: {len(paths)}')
        for path in paths:
            print('  ' + path)
    print('\nNo existing files will be overwritten; source packages and maps may be shadowed in the test overlay.')


def main():
    if len(sys.argv) == 3 and sys.argv[1] == 'scan':
        print(json.dumps(scan(sys.argv[2]), sort_keys=True))
    elif len(sys.argv) == 4 and sys.argv[1] == 'compare':
        with open(sys.argv[2], encoding='utf-8') as local, open(sys.argv[3], encoding='utf-8') as remote:
            compare(json.load(local), json.load(remote))
    else:
        raise ValueError('usage: inspect_sources.py scan ROOT | compare LOCAL_JSON REMOTE_JSON')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError) as error:
        print(f'inspection failed: {error}', file=sys.stderr)
        sys.exit(1)
