"""Operator attestation, not an automated or cryptographic safety certification."""
import argparse
import datetime
import json
from pathlib import Path
from .core import CHECKS, load_profile, approval_binding


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('profile'); parser.add_argument('output')
    args = parser.parse_args()
    p = load_profile(args.profile, 'bench')
    output = Path(args.output)
    if output.exists(): raise SystemExit('refusing to overwrite an existing report')
    print('Only attest tests actually completed on this exact vehicle/software. Abort on failure.')
    operator = input('Operator name: ').strip()
    if not operator: raise SystemExit('operator is required')
    for check in CHECKS:
        if input(check+' (type PASS only after physical test): ').strip() != 'PASS':
            raise SystemExit('not approved')
    report = {'operator': operator, 'date': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'binding': approval_binding(p), 'checks': {k: True for k in CHECKS}}
    with output.open('x') as stream: json.dump(report, stream, indent=2); stream.write('\n')
    print('Written:', output, 'Set drive_approval in profile to this absolute path.')


if __name__ == '__main__': main()
