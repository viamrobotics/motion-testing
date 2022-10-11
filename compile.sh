#!/bin/bash

cd omplbindings
sh makelib.sh
cd ../build 
cmake .. 
make