

#!/bin/sh
set -e

# Create the Fuel cache directory if it doesn't exist
mkdir -p ~/.gz/fuel/fuel.gazebosim.org

# Download the "quadcopter teleop" world from Fuel using the full URL
gz fuel download -u https://fuel.gazebosim.org/1.0/openrobotics/worlds/quadcopter\ teleop

mv "/root/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/quadcopter teleop" \
   "/root/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/quadcopter_teleop"
