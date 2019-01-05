#!/bin/bash
# Script to redo actions needed to rebuild code from source location.
#
# Written by Andre Strobel, 01/05/2019
#

cd ..
cp -R ~/Daten/DatenAMS/2_Archive/B_Programs/GH_GitHub/CarND-Path-Planning-Project/ .
cd CarND-Path-Planning-Project/
dos2unix *
./clean.sh
./build.sh
