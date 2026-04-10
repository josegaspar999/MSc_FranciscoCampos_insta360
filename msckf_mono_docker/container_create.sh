#! /bin/sh

# Include shared variables
source ./vars.sh

docker run -it \
    --network=host \
    -v $shared_volume:/root/cave:rw \
    --name=$container_name \
    $image_name:latest