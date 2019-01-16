#!/bin/bash
# Script to redo actions needed to rebuild code from source location.
#
# Written by Andre Strobel, 01/05/2019
#

cp ~/Daten/DatenAMS/2_Archive/B_Programs/GH_GitHub/CarND-Path-Planning-Project/CMakeLists.txt .
cp ~/Daten/DatenAMS/2_Archive/B_Programs/GH_GitHub/CarND-Path-Planning-Project/*.sh .
cp ~/Daten/DatenAMS/2_Archive/B_Programs/GH_GitHub/CarND-Path-Planning-Project/src/* ./src
dos2unix *
./clean.sh
./build.sh
