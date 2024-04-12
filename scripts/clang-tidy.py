#!/usr/env/bin python3

import json
import subprocess
import re
import argparse
import yaml

def _load_config() -> dict:
    """
    Loads the config off of disk. This is required especially for older versions of clang-tidy that do not support the new format, which
    allows for a nice list of checks instead of a big long comma delimited string
    """
    with open('.clang-tidy', 'r') as fp:
        f = yaml.safe_load(fp)
        f['Checks'] = ','.join(f['Checks']) # Convert from a list to a comma delimited str
        return f


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--clang-tidy', type=str, dest='clang_tidy', default='clang-tidy', help='Clang tidy executable to use')
    args = parser.parse_args()

    r = subprocess.run([args.clang_tidy, '--version'], capture_output=True)
    if r.returncode != 0:
        print(f'{args.clang_tidy} not found')
        exit(1)

    exclude = []
    with open('.clang-tidy-exclude', 'rb') as fp:
        exclude = yaml.safe_load(fp)['Exclude']

    assert isinstance(exclude, list)

    def is_excluded(file: str) -> bool:
        for reg in exclude:
            if re.search(reg, file) is not None:
                return True
        return False

    conf = json.dumps(_load_config())

    returnCode = 0
    with open('compile_commands.json', 'rb') as fp:
        f = json.load(fp)
        for cmd in f:
            file = cmd['file']
            if not is_excluded(file):
                print(f'Running clang-tidy on {file}')
                r = subprocess.run([args.clang_tidy, f'--config={conf}', file], cwd=cmd['directory'])
                if r.returncode != 0:
                    returnCode = r.returncode
            else:
                print(f'Skipped {file}')
    exit(returnCode)
