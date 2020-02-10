#!/bin/bash

### Run this script once you have the "interface" ethernet plugged in.
### Once it's run, plug the interface ethernet into the odroid, turn on the odroid,
### wait, and finally run
### >>>>>>>>>> tail -f /var/log/syslog >>>>>>>>>>
### to make sure the connection is working

# bring up the enp8s0 interface
sudo ifup enp8s0
# start DHCP service
sudo service isc-dhcp-server start # sudo service isc-dhcp-server stop
# enable forwarding from the ethernet to wireless router
sudo /sbin/iptables --table nat -A POSTROUTING -o wlan0 -j MASQUERADE
