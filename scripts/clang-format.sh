#!/bin/bash 

CLANG_FMT=$(which clang-format-14 || echo "clang-format")

$CLANG_FMT -i $(find . -iname "*.cpp" -o -iname "*.h")
