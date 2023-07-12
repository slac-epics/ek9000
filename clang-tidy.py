#!/usr/env/bin python3

import json
import subprocess
import re

if __name__ == '__main__':
    exclude = []
    with open('.clang-tidy-exclude.json', 'rb') as fp:
        exclude = json.load(fp)

    assert isinstance(exclude, list)

    def is_excluded(file: str) -> bool:
        for reg in exclude:
            if re.search(reg, file) is not None:
                return True
        return False

    returnCode = 0
    with open('compile_commands.json', 'rb') as fp:
        f = json.load(fp)
        for cmd in f:
            file = cmd['file']
            if not is_excluded(file):
                print(f'Running clang-tidy on {file}')
                r = subprocess.run(['clang-tidy', file], cwd=cmd['directory'])
                if r.returncode != 0:
                    returnCode = r.returncode
            else:
                print(f'Skipped {file}')
    exit(returnCode)
