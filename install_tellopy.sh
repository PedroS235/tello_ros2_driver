#!/bin/bash

git clone https://github.com/hanyazou/TelloPy.git tellopy
cd tellopy
sudo python3 setup.py install
cd ..
sudo rm -rf tellopy
