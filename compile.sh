#!/bin/bash

cd omplbindings
go mod tidy
go build -buildmode=c-shared -o bindings bindings.go scenes.go
cd ../build 
cmake .. 
make