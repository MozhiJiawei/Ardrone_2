#!/bin/bash
g++ `pkg-config opencv --cflags` usbcam.cpp  -o main `pkg-config opencv --libs`
