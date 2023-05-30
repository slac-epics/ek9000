#!/usr/bin/env python3

import json
import argparse
import jsonschema
import os

# Reference: https://json-schema.org/specification.html
SCHEMA = {
    'type': 'object',
    'properties': {
        'terminals': {
            'type': 'array',
            'items': {
                'type': 'object',
                'required': ['name', 'type', 'inputs', 'outputs', 'pdo_in_size', 'pdo_out_size'],
                'properties': {
                    'name': {
                        'type': 'string'
                    },
                    'type': {
                        'type': ['string', 'array'],
                        'if': {
                            'type': 'string'
                        },
                        'then': {
                            'enum': ['AnalogIn', 'AnalogOut', 'DigIn', 'DigOut', 'PositionMeasurement', 'DigInMulti', 'DigOutMulti']
                        },
                        'else': {
                            'items': {
                                'type': 'string',
                                'enum': ['AnalogIn', 'AnalogOut', 'DigIn', 'DigOut', 'PositionMeasurement', 'DigInMulti', 'DigOutMulti']
                            },
                        }
                    },
                    'inputs': {
                        'type': 'number',
                        'minimum': 0
                    },
                    'outputs': {
                        'type': 'number',
                        'minimum': 0
                    },
                    'pdo_in_size': {
                        'type': 'number',
                        'minimum': 0
                    },
                    'pdo_out_size': {
                        'type': 'number',
                        'minimum': 0
                    },
                    'dtyp': {
                        'type': 'string'
                    }
                }
            }
        }
    }
}


def get_scripts_dir() -> str:
    return os.path.dirname(__file__)


def get_db_dir() -> str:
    return f'{get_scripts_dir()}/../../Db'


def check_json(file: dict) -> bool:
    print('Validating terminals.json...', end='')
    jsonschema.validate(file, SCHEMA)
    print('OK')
    return True


def check_db(file: dict) -> bool:
    print('Checking for .template and .substitutions...', end='')
    ok = True
    for term in file['terminals']:
        name = term['name']
        files = [f'{name}.template', f'{name}.substitutions']
        for f in files:
            if not os.path.exists(f'{get_db_dir()}/{f}'):
                print(f'Missing {f}')
                ok = False
    print(f'{"OK" if ok == True else "FAILED"}')
    return ok    


def main():
    checks = {
        'db': check_db,
        'json': check_json
    }
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--checks', dest='CHECKS', nargs='+', default=checks.keys(), choices=checks.keys(), help='Checks to run')
    args = parser.parse_args()
    
    terms = {}
    with open(f'{get_scripts_dir()}/terminals.json', 'r') as fp:
        terms = json.load(fp)
    
    result = True
    for a in args.CHECKS:
        if not checks[a](terms):
            result = False

    if not result:
        exit(1)
    print('All checks passed')
    
if __name__ == '__main__':
    main()
