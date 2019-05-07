from __future__ import print_function
import rospy

import threading
import Queue

from alicat_ros.srv import SetFlowRate 
from alicat_ros.msg import DeviceSetPoint


class AlicatProxyException(Exception):
    pass


class AlicatProxy(object):

    def __init__(self,namespace=None,use_thread=True):

        self.namespace = namespace
        self.use_thread = use_thread
        self.service_name = 'alicat_set_flow_rate'

        if self.namespace is not None:
            self.service_name = '/{}/{}'.format(self.namespace,self.service_name)
        rospy.wait_for_service(self.service_name)
        
        if self.use_thread:
            self.proxy_queue = Queue.Queue()
            self.proxy_thread = threading.Thread(target=self.proxy_target)
            self.proxy_thread.daemon = True
            self.proxy_thread.start()
        else:
            self.set_flow_rate_proxy = rospy.ServiceProxy(self.service_name,SetFlowRate)

    def set_flow_rate(self,set_point_dict):
        set_point_list = []
        for addr,rate in set_point_dict.items():
            set_point_list.append(DeviceSetPoint(addr,rate))
        #for item in set_point_list:
        #    print(item)
        if self.use_thread:
            self.proxy_queue.put(set_point_list)
        else:
            rsp = self.set_flow_rate_proxy(set_point_list)

    def proxy_target(self):
        set_flow_rate_proxy = rospy.ServiceProxy(self.service_name,SetFlowRate)
        while True:
            try:
                set_point_list = self.proxy_queue.get_nowait()
            except Queue.Empty:
                continue
            rsp = set_flow_rate_proxy(set_point_list)


# Testing
# ---------------------------------------------------------------------------------------
if __name__ == '__main__':

    import time

    def get_set_point_dict(addr, addr_list, value):
        set_point_dict = {}
        for addr_tmp in addr_list:
            if addr_tmp == addr:
                set_point_dict[addr_tmp] = value 
            else:
                set_point_dict[addr_tmp] = 0.0 
        return set_point_dict

    proxy = AlicatProxy()

    addr_list = ['A','B','C','D','E','F'] 
    rate_list = [1.0, 0.0]
    sleep_dt = 5.0

    for i, addr in enumerate(addr_list):
        for rate in rate_list:
            print('{}/{}, addr={}: settiing flow rate to {}'.format(i+1,len(addr_list),addr,rate))
            set_point_dict = get_set_point_dict(addr, addr_list, rate)
            proxy.set_flow_rate(set_point_dict)
            time.sleep(sleep_dt)
    print('done')




