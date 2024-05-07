#!/bin/bash

protoc --cpp_out=cpp --python_out=python --plugin=../../reactapp/node_modules/.bin/protoc-gen-ts_proto --ts_proto_out=ts *.proto