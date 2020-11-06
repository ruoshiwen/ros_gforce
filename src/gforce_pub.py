#!/home/hit605/.pyenv/versions/myo_grasp/bin/python
from gforce import GForceProfile, NotifDataType, DataNotifFlags, NotifDataLength
import time
import numpy as np
import rospy
from std_msgs.msg import Header
from ros_gforce.msg import EmgArray, ImuArray, Quaternion, Euler
import struct

def onData(data):
    ''' According to https://oymotion.github.io/gForceSDK/gForceSDK/
        unpack data from different notifications
    '''
    if data[0] == NotifDataType['NTF_EMG_ADC_DATA'] and len(data) == NotifDataLength['NTF_EMG_ADC_LEN'] + 1:
        emg = data[1:]
        for i in range(16):
            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = 'gf_emg_' + str(i)
            emg_frame = list(struct.unpack('<8B',emg[8*i:8*i+8])) # the data type of the EMG samples is uint8_t
            # Wrap the EMG data into an EmgArray message
            emg_msg = EmgArray(h,emg_frame)
            emgPub.publish(emg_msg)        

    elif data[0] == NotifDataType['NTF_QUAT_FLOAT_DATA'] and len(data) == NotifDataLength['NTF_QUAT_FLOAT_LEN'] + 1:
        quat = data[1:]
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'gf_quat'
        quat_frame = struct.unpack('4f', quat)
        # Wrap the Quaternion data into a Quaternion message
        quat_msg = Quaternion()
        quat_msg.header = h
        quat_msg.data = quat_frame
        quatPub.publish(quat_msg)
    
    elif data[0] == NotifDataType['NTF_ACC_DATA'] and len(data) == NotifDataLength['NTF_ACC_LEN'] + 1:
        # 4 byte signed long
        acc = data[1:]
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'gf_acc'
        acc_frame = struct.unpack('<3l', acc)
        print(acc_frame)
        
        # Wrap the Accelerometer data into an ImuArray message
        acc_msg = ImuArray()
        acc_msg.header = h
        acc_msg.data = acc_frame
        accPub.publish(acc_msg)

    elif data[0] == NotifDataType['NTF_GYO_DATA'] and len(data) == NotifDataLength['NTF_GYO_LEN'] + 1:
        # 4 byte signed long
        gyro = data[1:]
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'gf_gyro'
        gyro_frame = struct.unpack('<3l', gyro)

        # Wrap the Gyroscope data into an ImuArray message
        gyro_msg = ImuArray()
        gyro_msg.header = h
        gyro_msg.data = gyro_frame
        gyroPub.publish(gyro_msg)

    elif data[0] == NotifDataType['NTF_EULER_DATA'] and len(data) == NotifDataLength['NTF_EULER_LEN'] + 1:
        euler = data[1:]
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'gf_euler'
        euler_frame = struct.unpack('3f', euler)
        
        # Wrap the Quaterion data into an Euler message
        euler_msg = Euler()
        euler_msg.header = h
        euler_msg.data = euler_frame
        eulerPub.publish(euler_msg)
    
def set_cmd_cb(resp,respdata):
    print('Command result: {}'.format(resp))

if __name__ == '__main__':
    # Config Emg Raw Data
    channel_mask = 0xFF
    data_len = 128
    samp_rate = rospy.get_param("~sample_rate", 650)
    resolution = rospy.get_param("~resolution", 8)
    
    emg_topic = rospy.get_param("~emg_topic", "gf_emg")
    acc_topic = rospy.get_param("~acc_topic", "gf_acc")
    
    quat_topic = rospy.get_param("~quat_topic", "gf_quat")
    
    gyro_topic = rospy.get_param("~gyro_topic", "gf_gyro")
    euler_topic = rospy.get_param("~euler_topic", "gf_euler")
    
    connected = 0
    while(connected==0):
        GF = GForceProfile()
        print('Scanning....')
        scan_results = GF.scan(5.0)
        if scan_results != []:
            try:
                addr = scan_results[0][2]
                print(addr)
                GF.connect(addr)  
                #GF.setEmgRawDataConfig(sampRate, channelMask, dataLen, resolution, cb=set_cmd_cb, timeout=5000)       
                connected = 1

            except (ValueError,KeyboardInterrupt) as e:
                print("gForce bracelet not found. Attempting to connect...")
                rospy.sleep(0.5)
                continue
    rospy.init_node('gf_pub_node')
    # Define the gforce publisher
    emgPub = rospy.Publisher(emg_topic, EmgArray, queue_size=20) # queue_size is set to 20 to guarantee the sampling rate
    accPub = rospy.Publisher(acc_topic, ImuArray, queue_size=10)
    
    quatPub = rospy.Publisher(quat_topic, Quaternion, queue_size=10)
    eulerPub = rospy.Publisher(euler_topic, Euler, queue_size=10)
    
    gyroPub = rospy.Publisher(gyro_topic, ImuArray, queue_size=10)
    
    
    
    GF.setDataNotifSwitch(DataNotifFlags['DNF_ACCELERATE'] | DataNotifFlags['DNF_EMG_RAW']
        | DataNotifFlags['DNF_QUATERNION'],set_cmd_cb,2000)
    try:
        GF.startDataNotification(onData) # startDataNotification gets one data packet, and uses ondata function to publish once
        rospy.spin()
    except (rospy.ROSInterruptException,KeyboardInterrupt) as e:
        pass
    finally:
        print("Disconnecting...")
        GF.stopDataNotification()
        GF.setDataNotifSwitch(DataNotifFlags['DNF_OFF'], set_cmd_cb, 2000)
        GF.disconnect()
