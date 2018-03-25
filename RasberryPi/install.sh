#!/bin/bash
#run this as root
#change hostname
echo $1 | cat > /etc/hostname
sed -i "s/raspberrypi/$1/g" /etc/hosts

#Setup SSH a little
mkdir -p /home/pi/.ssh
echo 'ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQCwiQfCRZL+rG8AtEeRk1eHSnZG3OtFap7bs1vqQ/SK4UdNjV3F+YRuW0OpM1DpffBW2Oiv/rEYu6CI/v7sXNlfhSBIOwlzs8fXfj5mYn4dKQWsOTYS9+GduVWy8N4e0uvd9kRyNKKz/EciRLz7B83C+XuHOtaGjxEQGGNuJx+LHo9rlXQXmmNUwFynGctwmJrPtY4hvoAXSMaNPbSdTZzHxskpVkmuq+Iu+0Ek1aVlYZdPPltGkwh7OBeHHmxZlt0HmjndHo3y0OlAmCjbO+LnYTTXrALx9NkGGJ/xtVDZ1DTnQxVSpGZblwGd7J1lLyPJnRsO9wHgIg1M+eLRebMh greg@nutonomy.com' | cat > /home/pi/.ssh/authorized_keys
echo 'alias temp="/opt/vc/bin/vcgencmd measure_temp"' | cat >> /home/pi/.bashrc
echo 'alias leases="cat /var/lib/dhcp/dhcpd.leases"' | cat >> /home/pi/.bashrc
raspi-config --expand-rootfs
apt update
apt upgrade -y
apt dist-upgrade
apt install -y rpi-update
rpi-update
apt install gpsd-clients -y

apt install -y ca-certificates
apt install -y isc-dhcp-server
apt install -y vim

sed -i "s/#authoritative;/authoritative;/g" /etc/dhcp/dhcpd.conf
sed -i "s/#net.ipv4.ip_forward=1/net.ipv4.ip_forward=1/g" /etc/sysctl.conf

#Config DHCP Server
cat >> /etc/dhcp/dhcpd.conf <<EOF
subnet 192.168.2.0 netmask 255.255.255.0 {
range 192.168.2.1 192.168.2.250;
option broadcast-address 192.168.2.255;
option routers 192.168.2.254;
default-lease-time 600;
max-lease-time 7200;
option domain-name "local";
option domain-name-servers 8.8.8.8, 8.8.4.4;
option interface-mtu 9000;
group {
	host douleia { hardware ethernet f8:ca:b8:25:f3:c4; fixed-address 192.168.2.253; } #test static ip assignment
}
}
EOF

#Config Network interfaces
cat > /etc/network/interfaces <<EOF
# interfaces(5) file used by ifup(8) and ifdown(8)

# Please note that this file is written to be used with dhcpcd
# For static IP, consult /etc/dhcpcd.conf and 'man dhcpcd.conf'

# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d

auto lo
iface lo inet loopback

#usb internet facing NIC
auto eth1
iface eth1 inet dhcp

#Ethernet NIC facing internal (lcm) network
auto eth0
iface eth0 inet static
address 192.168.2.254
netmask 255.255.255.0
network 192.168.2.0
broadcast 192.168.2.255
gateway 192.168.2.254

allow-hotplug wlan0
iface wlan0 inet manual
    wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf

allow-hotplug wlan1
iface wlan1 inet manual
    wpa-conf /etc/wpa_supplicant/wpa_supplicant.conf
EOF

#Make Internet on Command
cat > /home/pi/activate-internet.sh <<EOF
echo 1 > /proc/sys/net/ipv4/ip_forward
iptables -t nat -A POSTROUTING -o eth1 -j MASQUERADE
EOF

cat > /home/pi/disable-internet.sh <<EOF
echo 0 > /proc/sys/net/ipv4/ip_forward
iptables -t nat -A POSTROUTING -o eth1 -j MASQUERADE
iptables-restore < /etc/iptables.up.rules
EOF

chmod +x /home/pi/*-internet.sh

iptables -A INPUT -s 192.168.2.0/24 -i eth1 -j DROP
iptables -A INPUT -s 10.0.0.0/8 -i eth1 -j DROP
iptables -A INPUT -s 172.16.0.0/12 -i eth1 -j DROP
iptables -A INPUT -s 224.0.0.0/4 -i eth1 -j DROP
iptables -A INPUT -s 240.0.0.0/5 -i eth1 -j DROP
iptables -A INPUT -s 127.0.0.0/8 -i eth1 -j DROP
iptables -A INPUT -i eth1 -p icmp -m icmp --icmp-type 8 -j DROP
iptables-save > /etc/iptables.up.rules

#add iptables to network startup
cat > /etc/network/if-pre-up.d/iptables <<EOF
#!/bin/sh
#This script restores iptables upon reboot
iptables-restore < /etc/iptables.up.rules
/usr/sbin/gpsd -n -G /dev/ttyACM0
exit 0
EOF
sudo chmod +x /etc/network/if-pre-up.d/iptables

#Setting up GPSD
apt install -y pps-tools
apt install -y libcap-dev
apt install -y libssl-dev
#set GPIO pin for PPS
echo "dtoverlay=pps-gpio,gpiopin=18" | cat >> /boot/config.txt
echo "pps-gpio" | cat >> /etc/modules
apt install -y gpsd
cd /home/pi
wget http://git.tuxfamily.org/chrony/chrony.git/snapshot/chrony-master.tar.gz
tar zxvf chrony-master.tar.gz
apt install -y bison
apt install -y asciidoctor
cd /home/pi/chrony-master
./configure
sudo make install
#make install may fail at the end when it tries to compile documentation
#this is not a problem
cat > /etc/chrony.conf <<EOF
allow 192.168.2.0/24
refclock PPS /dev/pps0 lock GPS prefer refid PPS
refclock SHM 0 offset 0.1 delay 0.1 refid GPS noselect
driftfile /etc/chrony.drift
keyfile /etc/chrony.keys
leapsectz right/UTC
makestep 1.0 3
rtcsync
EOF

cat > /etc/rc.local <<EOF
#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

# Print the IP address
_IP=$(hostname -I) || true
if [ "$_IP" ]; then
  printf "My IP address is %s\n" "$_IP"
fi
/usr/local/sbin/chronyd
exit 0
EOF



