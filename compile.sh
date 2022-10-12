#!/bin/bash

cd omplbindings
go build -buildmode=c-shared bindings.go
cd ../build 
cmake .. 
make