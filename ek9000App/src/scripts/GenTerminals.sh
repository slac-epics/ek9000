#!/bin/bash
cd "$(dirname "$0")"
env python3 BhcParse.py -o "$(dirname $0)/../terminals.h" terminals.json 
