#!/bin/bash

#setup new heli

#fix locales
export LC_ALL=en_US.UTF-8
sudo locale-gen
sudo dpkg-reconfigure locales

# speed up boot if no network is present
sudo sed -i 's/sleep 59/sleep 0/' /etc/init/failsafe.conf
sudo sed -i 's/sleep 40/sleep 0/' /etc/init/failsafe.conf

# dont stop grub at problems
sudo sed -i 's/set timeout=${GRUB_RECORDFAIL_TIMEOUT:--1}/set timeout=1/' /etc/grub.d/00_header
sudo update-grub2

sudo apt-get install htop ssh vim ifplugd samba -y

source install_grub_hack.sh

source install_wpa.sh
