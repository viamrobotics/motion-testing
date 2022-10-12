#!/bin/bash

cd omplbindings
go mod tidy
go build -buildmode=c-shared bindings.go scenes.go
cd ../build 
cmake .. 
make