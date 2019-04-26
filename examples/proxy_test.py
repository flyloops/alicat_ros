from alicat_ros_proxy import AlicatProxy
import time

alicat_proxy = AlicatProxy()

for i in range(10):
    alicat_proxy.set_flow_rate({'A': 1.0, 'B': 0.0})
    time.sleep(5.0)
    alicat_proxy.set_flow_rate({'A': 0.0, 'B': 1.0})
    time.sleep(5.0)

alicat_proxy.set_flow_rate({'A': 0.0, 'B': 0.0})

