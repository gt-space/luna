#!/bin/bash

sleep 5

mkdir /mnt/blackbox_a
mkdir /mnt/blackbox_b

if ! mountpoint -q /mnt/blackbox_a; then
	mount /dev/sda1 /mnt/blackbox_a
fi 

if ! mountpoint -q /mnt/blackbox_b; then
	mount /dev/sdb1 /mnt/blackbox_b
fi
