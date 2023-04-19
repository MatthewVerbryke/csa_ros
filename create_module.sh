#!/bin/bash -e
# Copyright 2022 University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/MatthewVerbryke/csa_ros/LICENSE
# Additional copyright may be held by others, as reflected in the commit history.

# Get static paths
RELATIVE_PATH="`dirname \"$0\"`"
ABSOLUTE_PATH="`( cd \"$RELATIVE_PATH\" && pwd )`"
SRC_PATH=$(dirname "$ABSOLUTE_PATH")
CATKIN_PATH=$(dirname "$SRC_PATH")

# Get in name for new CSA module
echo "Name for the new module:"
read MODULE_NAME
echo ""

# Get location for new CSA module
echo "Location for the new module, starting from catkin workspace src directory (must be in this catkin package):"
read MODULE_LOCATION
echo ""

# Perform some basic analysis of the location for the directive
MODULE_PATH=$SRC_PATH/$MODULE_LOCATION
if [ -d "$MODULE_PATH" ];
then
    echo "$MODULE_PATH directory already exists"
else
    mkdir $MODULE_PATH && echo "Created new module top-level directory at $MODULE_PATH"
fi

# Create the basic structure of a ROS package
cd $MODULE_PATH/
catkin_create_pkg $MODULE_NAME std_msgs rospy
cd $MODULE_NAME/
touch README.md && echo "Created file $MODULE_NAME/README.md"

# Create and fill out launch directory
mkdir launch/ && echo "Created folder $MODULE_NAME/launch"
cd launch/
cp $SRC_PATH/csa_ros/csa_module/template/csa_launch_temp.txt $MODULE_NAME.launch && echo "Created file $MODULE_NAME/launch/$MODULE_NAME.launch"
cd ..

#  Create and fill out config directory
mkdir config/ && echo "Created folder $MODULE_NAME/config"
cd config/
cp $SRC_PATH/csa_ros/csa_module/template/csa_config_temp.txt config.yaml && echo "Created file $MODULE_NAME/config/config.yaml"
cd ..

# Create and fill out the source directory
cd src
mkdir $MODULE_NAME && echo "Created folder $MODULE_NAME/src/$MODULE_NAME"
cd $MODULE_NAME/
touch __init__.py && echo "Created file $MODULE_NAME/src/$MODULE_NAME/__init__.py"
cp $SRC_PATH/csa_ros/csa_module/template/csa_module_temp.txt $MODULE_NAME.py && echo "Created file $MODULE_NAME/src/$MODULE_NAME/$MODULE_NAME.py"
touch tactics_selection.py && echo "Created file $MODULE_NAME/src/$MODULE_NAME/tactics_selection.py"
mkdir tactics/ && echo "Created folder $MODULE_NAME/src/$MODULE_NAME/tactics/"
cd tactics/
touch __init__.py && echo "Created file $MODULE_NAME/src/$MODULE_NAME/tactics/__init__.py"

# Notify of end of program
echo ""
echo "Program finished. Check generated package to ensure everything was created okay"

