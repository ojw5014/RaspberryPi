#!/bin/bash
echo === compile ${1} ===
sudo g++ -o ${1} ${1}.cpp COjw_37_Protocol2.cpp COjw_37_Protocol2.h -lpthread
