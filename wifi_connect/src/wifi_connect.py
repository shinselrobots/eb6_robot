#!/usr/bin/env python3
# For headless robot (no monitor): Uses bluetooth phone (via Arduino) to connect to WIFI,
# Then allows boot of rest of the ROS stack.


import subprocess # for calling command line
import os
import signal
import time
import requests 

import rospy
import logging
from system_status_msgs.msg import SystemStatus
from behavior_msgs.msg import CommandState


class WifiConnect():

    def __init__(self):

        # init the node
        rospy.init_node('wifi_connect_node')
        rospy.loginfo("Starting wifi_connect_node...")
        self.pid = 0
        self.internet_is_up = False
              

        #SUBSCRIBERS
        behavior_cmd_sub = rospy.Subscriber('/wifi_connect', CommandState, self.behavior_command_cb)
        
        # PUBLISHERS
        # send status updates back to bluetooth phone
        # Note - we currently don't send these to the system monitor, since they are just for bring-up
        self.phone_pub = rospy.Publisher('/phone_update', SystemStatus, queue_size=6)


        self.send_phone_update('WIFI_CONNECT', "READY")
        print()
        print("*** Wifi Connect node running. Waiting for command from Bluetooth Phone ***")
        print("    Listening on message /wifi_connect")
        print()


    def send_phone_update(self, item, status):
        # Send status update system monitor
        status_msg = SystemStatus()
        status_msg.item = item
        status_msg.status = status
        self.phone_pub.publish(status_msg)


    def connect_wifi(self, ssid, password):
        """Connects to a Wi-Fi network with the given SSID and password."""

        print()
        print("Attempting to connect to SSID: [%s] ..." % ssid)
        print()
        
        try:
            subprocess.run(["nmcli", "dev", "wifi", "connect", ssid, "password", password], check=True)
            print("Successfully connected to", ssid)
            self.send_phone_update('WIFI_CONNECT', "CONNECTED")

        except subprocess.CalledProcessError as e:
            print("Error connecting to Wi-Fi:", e)
            self.send_phone_update('WIFI_CONNECT', "FAILED")


    def check_for_internet_connection(self):
        internet_up = False
        for i in range(0,5):
            try:
                status = requests.get('https://www.google.com/').status_code
                print("wifi_connect: Internet status returned: ", status)
                if status == 200:
                    internet_up = True
                break
            except:
                print("wifi_connect: Internet status returned exception")
                pass

            if internet_up:
                break
            else:
                print("wifi_connect: Internet not connected. Retrying...")
                self.send_phone_update('WIFI_CONNECT', "RETRYING")
                time.sleep(0.5)
            
        if internet_up:
            current_ssid = self.get_current_ssid()
            
            print("wifi_connect: Internet is up, SSID = [%s]" % current_ssid)
            status_update = "[" + current_ssid + "] IS UP"
            self.send_phone_update('WIFI_CONNECT', status_update)

        else:
            print("wifi_connect: Internet not connected.")
            self.send_phone_update('WIFI_CONNECT', "INTERNET_DOWN")
    


    def get_current_ssid(self):
        # Get the SSID of currently connected network

        try:
            output = subprocess.check_output(["iwgetid", "-r"]).decode("utf-8").strip()
            return output
        except subprocess.CalledProcessError:
            pass
        return ""

    
    
    
        
    def behavior_command_cb(self, data):
        # print("wifi_connect:got command:", data)        

        command = data.commandState

        if command == "WIFI_CONNECT":
            ssid = data.param1.strip() # remove spaces
            pw = data.param2.strip()

            print("")
            rospy.loginfo("*********************************************")
            rospy.loginfo("wifi_connect: Got command: %s " % command)        
            rospy.loginfo("              Param1: %s, Param2: %s" % (ssid, pw))        
            rospy.loginfo("*********************************************")


            if ssid == "":
                # empty request, just see if internet is up
                self.send_phone_update('WIFI_CONNECT', "CHECKING_STATUS")
                self.check_for_internet_connection()

            elif pw == "":
                rospy.loginfo("wifi_connect: Blank Password - Ignoring command %s " % pw)
                self.send_phone_update('WIFI_CONNECT', "BLANK_PW")

            else:
                self.send_phone_update('WIFI_CONNECT', "CONNECTING")
                self.connect_wifi(ssid, pw)
                time.sleep(1.0)
                self.check_for_internet_connection()

            
        elif command == "ROS_START":
            self.send_phone_update('WIFI_CONNECT', "STARTING_ROS")

            # Start the process in a new shell
            process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', '/home/system/.local/bin/robot_boot2.sh; exec bash'])

            # Get the PID of the process (WRONG PID - TODO)
            self.pid = process.pid
            print("Started process. Process PID:", self.pid)        



        # TODO this does not work. Wrong PID from above.
        elif command == "ROS_STOP":
            pid_str = data.param1
            if pid_str != "" and pid_str != "x":
                self.pid = int(pid_str)
            
            print("Stopping ROS, PID = %d" % self.pid)
            if self.pid != 0:
                os.kill(self.pid, signal.SIGKILL) #or signal.SIGKILL , SIGINT, SIGTERM
            else:
                print("NO PID")
        


if __name__=='__main__':
    # capture SIGINT signal, e.g., Ctrl+C
    #signal.signal(signal.SIGINT, signal_handler)
    node = WifiConnect()
    rospy.spin() # keep process alive
    
    
    

