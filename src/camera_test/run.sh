#!/bin/bash
g++ `pkg-config opencv --cflags` usbc	am.cpp  -o main `pkg-config opencv --libs`
