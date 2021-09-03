#!/bin/bash 

clang-format-14 -i $(find . -iname "*.cpp" -o -iname "*.h")
