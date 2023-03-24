#!/bin/bash
cd ../mivins_core/build/
cmake .. -DAPP_TYPE=ros1 && make
make
make install
cd ../../mivins_ros1/
catkin_make
