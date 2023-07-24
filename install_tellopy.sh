#!/bin/bash

git clone https://github.com/hanyazou/TelloPy.git tellopy
cd tellopy
pip install .

cd ..
sudo rm -rf tellopy
