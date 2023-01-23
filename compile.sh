#!/bin/bash

go mod tidy
cd omplbindings
go build -buildmode=c-shared -o bindings bindings.go scenes.go
mkdir -p ../build
cp bindings ../build # TODO(rb): this line fixes an issue with macs and dyld, we might want ot restructure so its not necessary
cd ../build 
cmake .. 
make