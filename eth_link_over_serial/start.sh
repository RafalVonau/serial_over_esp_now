#!/bin/bash

sudo ./eth_over_serial -B 2000000 -d /dev/ttyUSB0

#echo 1 | sudo tee /proc/sys/net/ipv4/ip_forward
#sudo /sbin/iptables -t nat -A POSTROUTING -o sl0 -j MASQUERADE
#sudo /sbin/iptables -t nat -A POSTROUTING -s 10.0.0.0/24 -d 0.0.0.0/0 -j MASQUERADE
