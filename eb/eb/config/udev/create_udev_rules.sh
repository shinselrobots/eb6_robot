#!/bin/bash

echo "start copy rules to  /etc/udev/rules.d/"
sudo cp *.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finished "
echo "check results using the command : ls -la /dev/ttyA* (ACM) or U (USB)"
echo "--------------------"
ls -la /dev/ttyA*
ls -la /dev/ttyU*
echo "--------------------"
ls -la /dev | grep "\->"
echo "--------------------"
echo "WARNING:  You might need to unplug/replug devices for links to show up (or reboot the pc)"


