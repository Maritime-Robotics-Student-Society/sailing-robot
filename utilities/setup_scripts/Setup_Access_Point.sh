#!/bin/sh 

# Install needed packages
sudo apt-get --assume-yes install hostapd udhcpd

# Copy default udhcpd config file in case needed
sudo cp /etc/udhcpd.conf /etc/udhcpd.conf.old

# Delete old and create new config file
sudo rm /etc/udhcpd.conf
sudo touch /etc/udhcpd.conf

# Contents of udhcpd config file
sudo echo "start 192.168.42.2" >> /etc/udhcpd.conf 		# This is the range of IPs that the hostspot will give to client devices.
sudo echo "end 192.168.42.20" >> /etc/udhcpd.conf
sudo echo "interface wlan0" >> /etc/udhcpd.conf 			# The device uDHCP listens on.
sudo echo "remaining yes" >> /etc/udhcpd.conf
sudo echo "opt dns 8.8.8.8 4.2.2.2" >> /etc/udhcpd.conf  # The DNS servers client devices will use.
sudo echo "opt subnet 255.255.255.0" >> /etc/udhcpd.conf
sudo echo "opt router 192.168.42.1" >> /etc/udhcpd.conf 	# The Pi's IP address on wlan0 which we will set up shortly.
sudo echo "opt lease 864000" >> /etc/udhcpd.conf 		# 10 day DHCP lease time in seconds

# Copy default udhcpd file in case need 
sudo cp /etc/default/udhcpd /etc/default/udhcpd.old

# Delete old and create new default udhcpd file
sudo rm /etc/default/udhcpd
sudo touch /etc/default/udhcpd

# Contents of default udhcpd file
sudo echo "DHCPD_OPTS=\"-S\"" >> /etc/default/udhcpd		# Log to syslog

# Give RPi static IP address
sudo ifconfig wlan0 192.168.42.1

# Copy default interfaces file in case needed
sudo cp /etc/network/interfaces /etc/network/interfaces.old

# Delete old and create new interfaces file
sudo rm /etc/network/interfaces
sudo touch /etc/network/interfaces

# Contents of interfaces file
sudo echo "source-directory /etc/network/interfaces.d" >> /etc/network/interfaces	# Include files from /etc/network/interfaces.d
sudo echo "allow-hotplug lo" >> /etc/network/interfaces
sudo echo "iface lo inet loopback" >> /etc/network/interfaces
sudo echo "auto wlan0" >> /etc/network/interfaces
sudo echo "allow-hotplug wlan0" >> /etc/network/interfaces
sudo echo "iface wlan0 inet static" >> /etc/network/interfaces
sudo echo "address 192.168.42.1" >> /etc/network/interfaces
sudo echo "netmask 255.255.255.0" >> /etc/network/interfaces
sudo echo "allow-hotplug eth0" >> /etc/network/interfaces
sudo echo "iface eth0 inet manual" >> /etc/network/interfaces

# Create HostAPD config file
sudo touch /etc/hostapd/hostapd.conf

# Contents of hostapd config file
sudo echo "interface=wlan0" >> /etc/hostapd/hostapd.conf
sudo echo "driver=nl80211" >> /etc/hostapd/hostapd.conf
sudo echo "ssid=Sail_Robot" >> /etc/hostapd/hostapd.conf
sudo echo "hw_mode=g" >> /etc/hostapd/hostapd.conf
sudo echo "channel=6" >> /etc/hostapd/hostapd.conf
sudo echo "macaddr_acl=0" >> /etc/hostapd/hostapd.conf
sudo echo "auth_algs=1" >> /etc/hostapd/hostapd.conf
sudo echo "ignore_broadcast_ssid=0" >> /etc/hostapd/hostapd.conf
sudo echo "wpa=2" >> /etc/hostapd/hostapd.conf
sudo echo "wpa_passphrase=sailrobot" >> /etc/hostapd/hostapd.conf
sudo echo "wpa_key_mgmt=WPA-PSK" >> /etc/hostapd/hostapd.conf
sudo echo "rsn_pairwise=CCMP" >> /etc/hostapd/hostapd.conf
# Commands below specific to RPi 3
sudo echo "ieee80211n=1" >> /etc/hostapd/hostapd.conf
sudo echo "wmm_enabled=1" >> /etc/hostapd/hostapd.conf


# Copy default hostapd file in case needed
sudo cp /etc/default/hostapd /etc/default/hostapd.old

# Delete old and create new interfaces file
sudo rm /etc/default/hostapd
sudo touch /etc/default/hostapd

# Contents of default hostapd file
sudo echo "DAEMON_CONF=\"/etc/hostapd/hostapd.conf\"" >> /etc/default/hostapd

# Configure NATing 
sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
sudo echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf

# Enable NAT in kernel
sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT

sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"
sudo echo "up iptables-restore < /etc/iptables.ipv4.nat" >> /etc/network/interfaces

# Start Access point
sudo service hostapd start
sudo service udhcpd start

# Start hotspot on boot
sudo update-rc.d hostapd enable
sudo update-rc.d udhcpd enable
