#!/bin/bash

go mod tidy
cd omplbindings
go build -buildmode=c-shared -o bindings bindings.go scenes.go
if [ ! -d ../build ]; then
  mkdir -p ../build;
fi
cd ../build 
cmake .. 
make