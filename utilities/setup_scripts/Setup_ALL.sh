#!/bin/bash 
set -e

if [ "$1" = '' ]
then
    echo -e "use with: \n $0 raspi \n or \n $0 workstation"
    exit 1
fi
    

setup () {
    echo
    echo "============== " $1 " =============="
    echo "Proceeding? [Y/n]"
    read ans
    if [ "$ans" = "y" ] || [ "$ans" = "Y" ] || [ "$ans" = '' ]
    then
        echo "== Running " $2
        ./$2
    else 
        echo "== Skipping"
    fi
}




setup "Install needed packages" Install_needed_packages.sh

if [ "$1" = "raspi" ]
then
    setup "Access Point" Setup_Access_Point.sh
fi
    
if [ "$1" = "raspi" ]
then
    setup "Raspberry pi setup" Install_needed_packages_Raspi.sh
fi

setup "Setting up ROS (ubuntu)" Setup_ROS.sh

if [ "$1" = "raspi" ]
then
    setup "Basic configuration (vim/bash)" Setup_basic_configuration.sh
fi
