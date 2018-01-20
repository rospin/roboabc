#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
import serial, struct, platform, math

class BaseDriver(object):
    
    BGN = '\xEE\xFF'
    END = '\x00\xBB'
    CMD = '\xEE\xAA'+'\x01\x00\x00'+'\x01\x00\x00'+'\x01\x00\x00'+'\x01\x00\xBB'
    
    def __init__(self):
        # initialize node
        rospy.init_node("base_driver")
        rospy.on_shutdown(self.shutdown)
        
        # establish communication
        self.comm = None
        self.comm = serial.Serial(rospy.get_param("~port", "/dev/ttyUSB0"), 
                                  rospy.get_param("~baud", 57600), timeout=1)

        # create subscriber, publisher and tf brodcaster
        rospy.Subscriber("cmd_vel", Twist, self.callback)
        odomPUB = rospy.Publisher("odom", Odometry, queue_size=10)
        odomTFB = tf.TransformBroadcaster()
        
        # get parameters of rate and frame_id
        rateCtrl = rospy.Rate(rospy.get_param("~rate",10.0))
        odom_FID = rospy.get_param('~odom_frame_id', 'odom')
        base_FID = rospy.get_param('~base_frame_id','base_footprint')
        
        #
        rospy.loginfo("BaseDriver running ...")
        
        # prepare data structure
        odometry = Odometry()
        odometry.header.frame_id = odom_FID
        odometry.child_frame_id  = base_FID
        odom_tf_ = TransformStamped()
        odom_tf_.header.frame_id = odom_FID
        odom_tf_.child_frame_id  = base_FID
        
        # initialize data
        buff = ""
        x, y, th, v, w = 0, 0, 0, 0, 0
        now = rospy.Time.now()
        
        # infinate loop
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            
            # read communation data buffer
            buff += self.comm.read(1)
            
            # locate and check valid data
            iBGN = buff.find(self.BGN)
            iDAT = iBGN +len(self.BGN)
            if iBGN>=0 and len(buff[iDAT:])>=16:
                iEND = iBGN + 16
                if buff[iEND:iEND+len(self.END)]==self.END:
                    # get valid data frame
                    data = buff[iDAT:iDAT+10]
                    buff = ""
                    
                    # prase data
                    x, y, th, v, w = struct.unpack(">5h", data)
                    x /= 100.0; y /= 100.0; th = th*math.pi/180.0
                    v /= 100.0; w /= 100.0
                    q = Quaternion(*tf.transformations.
                                    quaternion_from_euler(0,0,th))
                
                    # publish odometry msg
                    odometry.header.stamp = now
                    odometry.pose.pose.position.x  = x
                    odometry.pose.pose.position.y  = y
                    odometry.pose.pose.position.z  = 0
                    odometry.pose.pose.orientation = q
                    odometry.twist.twist.linear.x  = v
                    odometry.twist.twist.linear.y  = 0
                    odometry.twist.twist.angular.z = w
                    odomPUB.publish(odometry)
                   
                    # brodcast tf between odomtery and base
                    odom_tf_.header.stamp = rospy.Time.now()
                    odom_tf_.transform.translation.x = x
                    odom_tf_.transform.translation.y = y
                    odom_tf_.transform.translation.z = 0
                    odom_tf_.transform.rotation      = q
                    odomTFB.sendTransformMessage(odom_tf_)
                    
                    # clear communication input buffer
                    self.comm.flushInput() 
                else:
                    buff = ""
                    continue 
            else:
                continue
            
            rateCtrl.sleep()
        
    def callback(self, msg):
        # prase message data
        v = msg.linear.x
        w = msg.angular.z
        
        rospy.loginfo("CMD_VEL: v=%f, w=%f" % (v*1000.0, w*180.0/math.pi))
         
        vsig, vabs = (0x01, v) if v>0 else (0x02, -v)
        vabs = vabs*1000   # m/s --> mm/s
        wsig, wabs = (0x01, w) if w>0 else (0x02, -w)
        wabs = wabs*180/math.pi # rad/s --> deg/s
        
        # construct data frame and send it
        data = struct.pack(">3BhBh", 0xEE, 0xAA, vsig, vabs, wsig, wabs)
        data = data+self.CMD[len(data):]
        self.comm.write(data)
    
    def shutdown(self):
        # send stop command and close communication
        if self.comm!=None and self.comm.isOpen(): 
            self.comm.write(self.CMD)
            self.comm.close()
        
        # 
        rospy.sleep(1)
        rospy.loginfo("BaseDriver stopped.")
        
if __name__ == "__main__":
   try:
        BaseDriver()
   except Exception, e:
        pass
    
    
