#!/bin/bash

THIS_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")

. .venv/bin/activate

cd $THIS_DIR/src/cpp

make

cd $THIS_DIR

python3 -m src.safe_region_compare