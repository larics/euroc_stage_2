#!/bin/bash

sudo cp 09_config_serial /etc/grub.d
sudo chmod +x /etc/grub.d/09_config_serial # make it executable
sudo update-grub # rebuild grub config
