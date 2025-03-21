#!/usr/bin/env python3

"""
Generate st.cmd from a list of terminals, or the coupler's modbus configuration
"""

import os
import sys
import re
import argparse
import string

use_modbus = True
try:
	from pymodbus.client import ModbusTcpClient
except ImportError:
	print('Failed to import pymodbus, terminals must be manually specified on the command line')
	use_modbus = False

parser = argparse.ArgumentParser()
parser.add_argument('--ip', type=str, required=True, help='IP address and port the coupler can be reached at')
parser.add_argument('--out', type=str, default='st.cmd', help='File to output to')
parser.add_argument('--terminals', type=str, help='Comma separated list of terminals')
parser.add_argument('--port', type=int, default=502, help='Port to use')
parser.add_argument('--legacy', action='store_true', help='Generate a legacy rail configuration')
parser.add_argument('--ek9k-name', dest='ek9k_name', type=str, default='EK9K1', help='EK9000 device name, for the record')
parser.add_argument('--ioc-prefix', dest='prefix', type=str, default='iocEK9KTest', help='Prefix for the IOC')
parser.add_argument('--record-base', dest='record_base', type=str, default='t', help='Root of terminal record names')
args = parser.parse_args()

os.chdir(os.path.dirname(__file__))

arch = os.getenv('EPICS_HOST_ARCH')
if arch is None:
	print('EPICS_HOST_ARCH environment variable not set')
	sys.exit(1)

terms = []
if use_modbus and (args.terminals is None or len(args.terminals) == 0):
	client = ModbusTcpClient(args.ip, args.port)
	if not client.connect():
		print('Connection to coupler failed, use --terminals instead')
		sys.exit(1)
	resp = client.read_holding_registers(0x6001, 125)
	for r in resp.registers:
		if r == 0:
			break
		name = f'EL{r}'
		if not os.path.exists(f'../../db/{name}.db'):
			print(f'Unsupported terminal type {name}, stopping here!')
			break
		terms.append(f'EL{r}')
	print(f'Determined layout from coupler: {terms}')
	client.close()
else:
	terms = args.terminals.split(',')

with open('st.tpl', 'r') as fp:
	data = fp.read()

data = string.Template(data).safe_substitute({
	'PREFIX': args.prefix,
	'EK9K': args.ek9k_name,
	'NUM_TERMS': str(len(terms)),
	'IP': args.ip,
	'ARCH': arch,
	'PORT': str(args.port)
})

conf = ''
if args.legacy:
    for index, term in enumerate(terms, start=1):
    	conf += f'ek9000ConfigureTerminal("EK9K1", "t{index}", "{term}", {index})\n'
data = data.replace('$CONFIGURE$', conf)

rec = ''
for index, term in enumerate(terms, start=1):
	rec += f'dbLoadRecords("../../db/{term}.db", "TERMINAL={args.record_base}{index},DEVICE={args.ek9k_name},POS={index}")\n'
data = data.replace('$RECORDS$', rec)

with open(args.out, 'w') as outfp:
	outfp.write(data)
os.chmod(args.out, 0o755)
