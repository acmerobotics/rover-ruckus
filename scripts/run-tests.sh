#!/bin/bash

set -ev
cd ..
git clone https://github.com/acmerobotics/ftc_app --depth=1
cd ftc_app
./gradlew.sh build