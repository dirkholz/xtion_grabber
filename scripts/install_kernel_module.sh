#!/bin/sh

module_git=https://github.com/dirkholz/xtion.git
xtion_grabber=$(rospack find xtion_grabber)
module_dir=${xtion_grabber}/kernel_module

if [ ! -d "${module_dir}" ]; then
    echo "Retrieving xtion repository"
    mkdir -p ${module_dir} && git clone ${module_git} ${module_dir} && git checkout develop
else
    echo "Directory  ${module_dir} already exists. Updating ..."
    cd ${module_dir} && git pull
fi

echo "Compiling and installing the xtion kernel module"
# cd ${module_dir} && make modules && sudo make modules_install && sudo modprobe -r xtion && sudo depmod -a && sudo modprobe xtion
cd ${module_dir} && make modules && sudo make modules_install && sudo depmod -a && sudo modprobe xtion

udev_file="/etc/udev/rules.d/556-xtion-kernel-cameras.rules"
if [ ! -e ${udev_file} ]; then
    echo "Creating udev rule ${udev_file}"
    echo '# Rules for the xtion kernel driver' | sudo tee --append ${udev_file} > /dev/null
    echo 'SUBSYSTEM=="video4linux", ATTR{xtion_endpoint}=="depth", ATTRS{xtion_id}=="*", SYMLINK+="xtion_$attr{xtion_id}_depth"' | sudo tee --append ${udev_file} > /dev/null
    echo 'SUBSYSTEM=="video4linux", ATTR{xtion_endpoint}=="color", ATTRS{xtion_id}=="*", SYMLINK+="xtion_$attr{xtion_id}_color"' | sudo tee --append ${udev_file} > /dev/null
fi


