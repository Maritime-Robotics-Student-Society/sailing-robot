#!/bin/sh

# Login as root
sudo -i

# Install needed packages
apt-get --assume-yes install hostapd udhcpd

# Copy default udhcpd config file in case needed
cp /etc/udhcpd.conf /etc/udhcpd.conf.old

# Delete old and create new config file
rm /etc/udhcpd.conf
touch /etc/udhcpd.conf

# Contents of udhcpd config file
echo "start 192.168.42.2" >> /etc/udhcpd.conf 		# This is the range of IPs that the hostspot will give to client devices.
echo "end 192.168.42.20" >> /etc/udhcpd.conf
echo "interface wlan0" >> /etc/udhcpd.conf 			# The device uDHCP listens on.
echo "remaining yes" >> /etc/udhcpd.conf
echo "opt dns 8.8.8.8 4.2.2.2" >> /etc/udhcpd.conf  # The DNS servers client devices will use.
echo "opt subnet 255.255.255.0" >> /etc/udhcpd.conf
echo "opt router 192.168.42.1" >> /etc/udhcpd.conf 	# The Pi's IP address on wlan0 which we will set up shortly.
echo "opt lease 864000" >> /etc/udhcpd.conf 		# 10 day DHCP lease time in seconds

# Copy default udhcpd file in case need 
cp /etc/default/udhcpd /etc/default/udhcpd.old

# Delete old and create new default udhcpd file
rm /etc/default/udhcpd
touch /etc/default/udhcpd

# Contents of default udhcpd file
echo "DHCPD_OPTS=\"-S\"" >> /etc/default/udhcpd		# Log to syslog

# Give RPi static IP address
ifconfig wlan0 192.168.42.1

# Copy default interfaces file in case needed
cp /etc/network/interfaces /etc/network/interfaces.old

# Delete old and create new interfaces file
rm /etc/network/interfaces
touch /etc/network/interfaces

# Contents of interfaces file
echo "source-directory /etc/network/interfaces.d" >> /etc/network/interfaces	# Include files from /etc/network/interfaces.d
echo "allow-hotplug lo" >> /etc/network/interfaces
echo "iface lo inet loopback" >> /etc/network/interfaces
echo "auto wlan0" >> /etc/network/interfaces
echo "allow-hotplug wlan0" >> /etc/network/interfaces
echo "iface wlan0 inet static" >> /etc/network/interfaces
echo "address 192.168.42.1" >> /etc/network/interfaces
echo "netmask 255.255.255.0" >> /etc/network/interfaces
echo "allow-hotplug eth0" >> /etc/network/interfaces
echo "iface eth0 inet manual" >> /etc/network/interfaces

# Create HostAPD config file
touch /etc/hostapd/hostapd.conf

# Contents of hostapd config file
echo "interface=wlan0" >> /etc/hostapd/hostapd.conf
echo "driver=nl80211" >> /etc/hostapd/hostapd.conf
echo "ssid=UoS_Sailing_Robot" >> /etc/hostapd/hostapd.conf
echo "hw_mode=g" >> /etc/hostapd/hostapd.conf
echo "channel=6" >> /etc/hostapd/hostapd.conf
echo "macaddr_acl=0" >> /etc/hostapd/hostapd.conf
echo "auth_algs=1" >> /etc/hostapd/hostapd.conf
echo "ignore_broadcast_ssid=0" >> /etc/hostapd/hostapd.conf
echo "wpa=2" >> /etc/hostapd/hostapd.conf
echo "wpa_passphrase=Autonomous_Sailing" >> /etc/hostapd/hostapd.conf
echo "wpa_key_mgmt=WPA-PSK" >> /etc/hostapd/hostapd.conf
echo "rsn_pairwise=CCMP" >> /etc/hostapd/hostapd.conf
# Commands below specific to Ri
echo "ieee80211n=1" >> /etc/hostapd/hostapd.conf
echo "wmm_enabled=1" >> /etc/hostapd/hostapd.conf
echo "ht_capab=[HT40][SHORT-GI-20][DSSS_CCK-40]" >> /etc/hostapd/hostapd.conf
# Copy default hostapd file in case needed
cp /etc/default/hostapd /etc/default/hostapd.old

# Delete old and create new interfaces file
rm /etc/default/hostapd

# Contents of default hostapd file
echo "DAEMON_CONF=\"/etc/hostapd/hostapd.conf\"" >> /etc/default/hostapd

# Configure NATing 
sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"
echo "net.ipv4.ip_forward=1" >> /etc/sysctl.conf

# Enable NAT in kernel
iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
iptables -A FORWARD -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
iptables -A FORWARD -i wlan0 -o eth0 -j ACCEPT

sh -c "iptables-save > /etc/iptables.ipv4.nat"
echo "up iptables-restore < /etc/iptables.ipv4.nat" >> /etc/network/interfaces

# Start Access point
service hostapd start
service udhcpd start

# Start hotspot on boot
update-rc.d hostapd enable
update-rc.d udhcpd enable

# Exit root
logout
