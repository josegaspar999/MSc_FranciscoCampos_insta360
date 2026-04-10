#!/bin/sh

# Include shared variables
source ./vars.sh

docker build . -t $image_name