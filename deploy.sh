#!/usr/bin/env bash

# --- [change these] ---

# local workspace
LOCAL_WS="/home/$USER/dev_ws"

# remote workspace
REMOTE_WS="csrobot@10.0.9.2:/home/csrobot/remote_ws"
# REMOTE_WS="/home/$USER/remote_ws"

# name of local ROS package
PACKAGE_NAME="scooter"

# ----------------------

SOURCE="${LOCAL_WS}/src/${PACKAGE_NAME}"
DESTINATION="${REMOTE_WS}/src"

SKIP_CONFIRM=0

# parse options
while getopts 'y' opt; do
    case $opt in
        y) SKIP_CONFIRM=1 ;;
        *) echo "Unrecognized option" >&2
           exit 1
    esac
done

# confirmation prompt
if [ $SKIP_CONFIRM == 0 ]; then
    # confirm
    read -p "This will overwrite ${DESTINATION}/${PACKAGE_NAME}, are you sure? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]
    then
        exit 1
    fi
fi

# rsync
rsync -avz $SOURCE $DESTINATION
