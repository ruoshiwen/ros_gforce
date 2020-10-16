#!/home/hit605/.pyenv/versions/myo_grasp/bin/python
from gforce import GForceProfile, NotifDataType, DataNotifFlags
import time
import os
import numpy as np
import rospy
from std_msgs.msg import Header
from ros_gforce.msg import EmgArray
import struct

DATA_LEN = 128
# An example of the callback function
def set_cmd_cb(resp,respdata):
    #print('Command result: {}'.format(resp))
    pass

# An example of the ondata
def ondata(data):
    if data[0] == NotifDataType['NTF_EMG_ADC_DATA'] and len(data) == DATA_LEN + 1:
        emg = data[1:DATA_LEN+1]
        for i in range(8):
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = 'gf_emg_' + str(i)
            emg_frame = []
            for ch in range(8):
#                print('channel bytes',emg[16*i+2*ch:16*i+2*ch+2])
#                print('channel', ch, 'high:', emg[16*i+2*ch+1],'low:',emg[16*i+2*ch])
                emg_frame.append(int.from_bytes(emg[16*i+2*ch:16*i+2*ch+2],byteorder= 'little',signed=False)) # store in LSB format, channel[0] = emg[1], emg[0]
            emg_msg = EmgArray()
            emg_msg.header = h
            emg_msg.data = emg_frame
            emgPub.publish(emg_msg)


if __name__ == '__main__':
    # Config emg raw data acquisition
    channelMask = 0xFF
    dataLen = 128
    sampRate = rospy.get_param("~sample_rate", 100)
    resolution = rospy.get_param("~resolution", 12)
    emg_topic = rospy.get_param("~emg_topic", "gf_emg")
    connected = 0
    print("Scanning...")
    while (connected==0):
        GF = GForceProfile()
        scan_results = GF.scan(5)
        if scan_results != []:
            try:
                addr = scan_results[0][2]
                GF.connect(addr) # setNotify:Listen cmd ，Turn notifications on by setting bit0 in the handle of characteristic ; 
                GF.setEmgRawDataConfig(sampRate, channelMask, dataLen, resolution, cb=set_cmd_cb, timeout=1000)       
                connected = 1
                print('gForce bracelet is configured.')
            except(ValueError,KeyboardInterrupt) as e:
                print("gForce bracelet not found. Attempting to connect...")
                time.sleep(0.5)
                continue
    GF.setDataNotifSwitch(DataNotifFlags['DNF_EMG_RAW'],set_cmd_cb,1000) # # Set data notification flag
    emgPub = rospy.Publisher(emg_topic, EmgArray, queue_size=10)
    rospy.init_node('gf_raw_node')
    try:
        GF.startDataNotification(ondata) # startDataNotification gets one data packet, and uses ondata function to publish once
        rospy.spin()
    except (rospy.ROSInterruptException,KeyboardInterrupt) as e:
        pass
    finally:
        print("Disconnecting...")
        GF.stopDataNotification()
        GF.disconnect()

