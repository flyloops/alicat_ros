# alicat_ros 

ROS interface to alicat mass flow controllers. Contatins a ROS node
"alicat_ros_node" which communicates via USB/serial with the mass flow
controllers connnect via the BB9-232 interface and provides a ROS service for
setting the mass flow rates. 

## Requirements

* [ROS](http://wiki.ros.org/Documentation) (tested with kinetic, desktop install)
* [pyserial](https://pythonhosted.org/pyserial/)
* [alicat](https://github.com/numat/alicat)  

## Installation

1. Install ROS. Instructions can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu) 
if you have not already done this.  Tested with ros kinetic, desktop install

2. Setup your catkin workspace.  Instructions can be found [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 
if you have not already done this. 
    
3. Download and install the python library from [here](https://github.com/numat/alicat)

4. Then clone the git repository and run catkin make.

```bash
$cd ~/catkin_ws/src
$git clone https://github.com/willdickson/alicat_ros.git
$cd ~/catkin_ws
$catkin_make

```

## Alicat proxy service example

```python
from alicat_ros_proxy import AlicatProxy
import time

alicat_proxy = AlicatProxy()

for i in range(10):
    alicat_proxy.set_flow_rate({'A': 1.0, 'B': 0.0})
    time.sleep(5.0)
    alicat_proxy.set_flow_rate({'A': 0.0, 'B': 1.0})
    time.sleep(5.0)

alicat_proxy.set_flow_rate({'A': 0.0, 'B': 0.0})
```



