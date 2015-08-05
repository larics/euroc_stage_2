#!/bin/bash

echo "Do you want to configure wpa_supplicant? [Y,n]"
read input
if [[ $input == "Y" || $input == "y" ]]; then
	echo ""
	echo "=== Setting up wpa_supplicant ==="
	read -p "your wifi-name? " wifi_name
	read -p "your wifi-password? " wifi_password

	sudo sh -c "wpa_passphrase $wifi_name $wifi_password >> /etc/wpa_supplicant/wpa_supplicant.conf"

	sudo sh -c 'echo "
	auto wlan0
	iface wlan0 inet dhcp
		wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
		post-up /sbin/iwconfig wlan0 power off  # makes sure power management is off --> short ping times" >> /etc/network/interfaces'
fi

echo "=== Setting up serial port ==="
sudo adduser $USER dialout

