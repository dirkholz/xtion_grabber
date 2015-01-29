#!/bin/sh

# module_git=https://github.com/xqms/xtion.git
module_git=https://github.com/dirkholz/xtion.git

xtion_grabber=$(rospack find xtion_grabber)
module_dir=${xtion_grabber}/kernel_module
if [ ! -d "${module_dir}" ]; then
    echo "Retrieving, compiling and installing the xtion kernel module"
    mkdir -p ${module_dir} && git clone ${module_git} ${module_dir} && cd ${module_dir} && make modules && sudo make modules_install && sudo depmod -a
else
    echo "WARNING: directory ${module_dir} already exists!" >> /dev/stderr
fi

udev_file="/etc/udev/rules.d/556-xtion-kernel-cameras.rules"
if [ ! -e ${udev_file} ]; then
    echo '# Rules for the xtion kernel driver' | sudo tee --append ${udev_file} > /dev/null
    echo 'SUBSYSTEM=="video4linux", ATTR{xtion_endpoint}=="depth", ATTRS{xtion_id}=="*", SYMLINK+="xtion_$attr{xtion_id}_depth"' | sudo tee --append ${udev_file} > /dev/null
    echo 'SUBSYSTEM=="video4linux", ATTR{xtion_endpoint}=="color", ATTRS{xtion_id}=="*", SYMLINK+="xtion_$attr{xtion_id}_color"' | sudo tee --append ${udev_file} > /dev/null
else
    echo "WARNING: udev rule ${udev_file} already exists!" >> /dev/stderr
fi


