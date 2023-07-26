#!/bin/bash
set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
if ! command -v xvfb &> /dev/null
then
    echo "xvfb could not be found. Installing now..."
    sudo apt-get install xvfb -y
else
    echo "xvfb is installed."
fi

# Check if Docker image exists
if [[ "$(docker images -q myimage 2> /dev/null)" == "" ]]; then
    echo "Docker image doesn't exist. Building now..."
    docker build -t myimage $DIR
else
    echo "Docker image already exists."
fi

xhost +

docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    myimage
