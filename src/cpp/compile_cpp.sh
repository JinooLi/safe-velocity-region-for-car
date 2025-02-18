#!/bin/bash

# 이 파일이 있는 디렉토리 확인
# 절대 경로 사용
THIS_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")

cd $THIS_DIR

c++ -O3 -Wall -shared -std=c++11 -fPIC \
    $(python3 -m pybind11 --includes) \
    src/pybind.cpp src/safecar.cpp -o safecar$(python3-config --extension-suffix)


