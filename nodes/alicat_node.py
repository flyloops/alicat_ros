#!/usr/bin/env python
from __future__ import print_function
import os
import threading
import yaml

import rospy
import std_msgs.msg

from alicat import FlowController
from alicat_ros.msg import Device
from alicat_ros.msg import DeviceArray
from alicat_ros.srv import SetFlowRate 
from alicat_ros.srv import SetFlowRateResponse

class AlicatNode(object):

    Default_Param_File = 'alicat_param.yaml'

    def __init__(self):
        self.lock = threading.Lock()
        rospy.init_node('alicat')
        self.get_param()
        self.rate = rospy.Rate(self.param['publish_rate'])

        self.controllers = {}
        for address in self.param['addresses']:
            self.controllers[address] = FlowController(self.param['port'], address, self.param['protocol']) 

        self.zero_flow_rates()

        self.device_array_pub = rospy.Publisher('alicat_device_array', DeviceArray, queue_size=10)
        self.set_flow_rate_srv = rospy.Service('alicat_set_flow_rate', SetFlowRate, self.on_set_flow_rate)



    def zero_flow_rates(self):
        """Set flow rates to zero. 
        """
        max_try_cnt = 5
        self.set_point_dict = {address:0.0 for address in self.param['addresses']}
        for address, rate in self.set_point_dict.items(): 
            ok = False
            cnt = 0
            while not ok and cnt < max_try_cnt:
                try:
                    self.controllers[address].set_flow_rate(rate)
                    ok = True
                except UnicodeDecodeError:
                    cnt+=1

    def on_set_flow_rate(self,req):
        success = True
        message = ''

        for set_point in req.set_points:
            self.set_point_dict[set_point.address] = set_point.rate

        for set_point in req.set_points:
            if not set_point.address in self.param['addresses']:
                success = False
                message = 'invalid device address'
                break

            try:
                with self.lock:
                    self.controllers[set_point.address].set_flow_rate(set_point.rate)
            except Exception, e:
                success = False 
                message = str(e)
                break

        return SetFlowRateResponse(success,message)

    def get_param(self):
        self.param = rospy.get_param('/alicat', None)
        if self.param is None:
            param_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.Default_Param_File)
            with open(param_file_path,'r') as f:
                self.param = yaml.load(f)

    def run(self):
        while not rospy.is_shutdown():
            # Read actual flow rate from alicat devices
            actual_dict = {address:None for address in self.param['addresses']}
            for address in self.param['addresses']:
                try:
                    with self.lock:
                        flow = self.controllers[address].get()['volumetric_flow'] 
                except Exception, e:
                    continue
                actual_dict[address] = flow

            # Publish flow data
            msg = DeviceArray()
            msg.header.stamp = rospy.Time.now()
            for address in self.param['addresses']:
                device_data = Device()
                device_data.address = address
                device_data.rate_set_point = self.set_point_dict[address]
                if actual_dict[address] is not None:
                    device_data.rate_actual = actual_dict[address]
                    device_data.success = True
                else:
                    device_data.rate_actual = 0.0 
                    device_data.success = False 
                msg.devices.append(device_data)
            self.device_array_pub.publish(msg) 

            self.rate.sleep()

# ------------------------------------------------------------------------------- 
if __name__ == '__main__':

    node = AlicatNode()
    node.run()

