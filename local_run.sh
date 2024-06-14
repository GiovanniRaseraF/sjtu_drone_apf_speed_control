#!/bin/bash
 
# credits: Giovanni Rasera and Diletta Giabardo


usage(){
    echo "Usage: $0 [-r <humble|iron|rolling>]"
    exit 1
}

# build your packages
colcon build --packages-select-regex sjtu_drone_bringup sjtu_drone_control sjtu_drone_description
echo "Done: buildi packages"
date

ROS_DISTRO=${ROS_DISTRO:-"humble"}  # [humble, iron, rolling]
while getopts "r:" opt; do
    case $opt in
        r)
            if [ $OPTARG != "humble" ] && [ $OPTARG != "iron" ] && [ $OPTARG != "rolling" ]; then
                echo "Invalid ROS distro: $OPTARG" >&2
                usage
            fi
            ROS_DISTRO=$OPTARG
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            ;;
    esac
done

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

if [ $? -ne 0 ]; then
    exit 1
fi

xhost +local:docker

# this will run your local container
docker run \
    -it --rm \
    $VOLUMES \
    -v ${XSOCK}:${XSOCK} \
    -v ${XAUTH}:${XAUTH} \
    -e DISPLAY=${DISPLAY} \
    -e XAUTHORITY=${XAUTH} \
    --env=QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    --name="sjtu_drone" \
    sjtu_drone

xhost -local:docker
