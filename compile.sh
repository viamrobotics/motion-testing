#!/bin/bash

go mod tidy
cd omplbindings
go build -buildmode=c-shared -o bindings bindings.go scenes.go
cd ../build 
cmake .. 
make