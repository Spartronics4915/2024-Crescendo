#!/usr/bin/bash

# Clears the deploy directory.
# Must be run on the target roboRIO.

if [[ ! $hostname =~ "roboRIO" ]]; then
    echo "Not a roboRIO!";
    exit 1;
fi

if [ ! -d /home/lvuser/deploy ]; then
    echo "Deploy directory not found!"
    exit 1;
fi

rm -r /home/lvuser/deploy
