'''
python file to dump messages to a json file that can be analysed
must run in container/virtual machine with ros installed
Create rosbag file with rosbag record -a
'''

import ros
import json

import rosbag
from std_msgs.msg import Int32, String

bag = rosbag.Bag('/udacity/CarND-TrackMasters-Capstone/data/2017-11-12-06-31-37.bag')

'''
[ INFO] [1510468297.878817543]: Subscribing to /base_waypoints
[ INFO] [1510468297.882706746]: Recording to 2017-11-12-06-31-37.bag.
[ INFO] [1510468297.935085074]: Subscribing to /final_waypoints
[ INFO] [1510468298.015050741]: Subscribing to /vehicle/steering_cmd
[ INFO] [1510468298.081321778]: Subscribing to /vehicle/brake_cmd
[ INFO] [1510468298.122575993]: Subscribing to /vehicle/obstacle_points
[ INFO] [1510468298.212750470]: Subscribing to /vehicle/lidar
[ INFO] [1510468298.293755729]: Subscribing to /vehicle/dbw_enabled
[ INFO] [1510468298.343508359]: Subscribing to /rosout
[ INFO] [1510468298.426234386]: Subscribing to /rosout_agg
[ INFO] [1510468298.499899528]: Subscribing to /current_pose
[ INFO] [1510468298.560384553]: Subscribing to /current_velocity
[ INFO] [1510468298.586866286]: Subscribing to /vehicle/obstacle
[ INFO] [1510468298.661616878]: Subscribing to /twist_cmd
[ INFO] [1510468298.707964697]: Subscribing to /vehicle/steering_report
[ INFO] [1510468298.732769022]: Subscribing to /traffic_waypoint
[ INFO] [1510468298.767035864]: Subscribing to /vehicle/throttle_cmd
[ INFO] [1510468298.835437480]: Subscribing to /vehicle/traffic_lights
[ INFO] [1510468298.902429847]: Subscribing to /vehicle/brake_report
[ INFO] [1510468298.916447325]: Subscribing to /tf
[ INFO] [1510468298.927338843]: Subscribing to /image_color
[ INFO] [1510468298.943482956]: Subscribing to /vehicle/throttle_report
'''

msgList = []


for topic, msg, t in bag.read_messages(topics=['/vehicle/steering_cmd']):
    #print('{0},{1}:steering_wheel_angle_cmd,{2}'.format(t,topic,msg.steering_wheel_angle_cmd))
    top = '{}:steering_wheel_angle_cmd'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.steering_wheel_angle_cmd}
    msgList.append(entry)



for topic, msg, t in bag.read_messages(topics=['/vehicle/brake_cmd']):
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('{0},{1}:pedal_cmd,{2}'.format(t,topic,msg.pedal_cmd))
    top = '{}:pedal_cmd'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.pedal_cmd}
    msgList.append(entry)

for topic, msg, t in bag.read_messages(topics=['/vehicle/throttle_cmd']):
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('{0},{1}:pedal_cmd,{2}'.format(t,topic,msg.pedal_cmd))
    top = '{}:pedal_cmd'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.pedal_cmd}
    msgList.append(entry)

for topic, msg, t in bag.read_messages(topics=['/current_velocity']):
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('{0},{1}:x,{2}'.format(t,topic,msg.twist.linear.x))
    top = '{}:twist.linear.x'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.twist.linear.x}
    msgList.append(entry)



for topic, msg, t in bag.read_messages(topics=['/twist_cmd']):
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('{0},{1}:x,{2}'.format(t,topic,msg.twist.linear.x))
    top = '{}:twist.linear.x'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.twist.linear.x}
    msgList.append(entry)

for topic, msg, t in bag.read_messages(topics=['/twist_cmd']):
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('{0},{1}:z,{2}'.format(t,topic,msg.twist.angular.z))
    top = '{}:twist.angular.z'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.twist.angular.z}
    msgList.append(entry)

for topic, msg, t in bag.read_messages(topics=['/vehicle/steering_report']):
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('{0},{1}:speed,{2}'.format(t,topic,msg.speed))
    top = '{}:speed'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.speed}
    msgList.append(entry)

for topic, msg, t in bag.read_messages(topics=['/vehicle/steering_report']):
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('{0},{1}:steering_wheel_angle_cmd,{2}'.format(t,topic,msg.steering_wheel_angle_cmd))
    top = '{}:steering_wheel_angle_cmd'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.steering_wheel_angle_cmd}
    msgList.append(entry)

for topic, msg, t in bag.read_messages(topics=['/vehicle/brake_report']):
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('{0},{1}:data,{2}'.format(t,topic,msg.data))
    top = '{}:data'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.data}
    msgList.append(entry)

for topic, msg, t in bag.read_messages(topics=['/vehicle/throttle_report']):
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('{0},{1}:data,{2}'.format(t,topic,msg.data))
    top = '{}:data'.format(topic)
    entry = {'t':t.to_nsec(),'topic':top,'value':msg.data}
    msgList.append(entry)

#for topic, msg, t in bag.read_messages(topics=['/vehicle/steering_cmd', '/vehicle/brake_cmd','/vehicle/throttle_cmd','/current_velocity','/twist_cmd','/vehicle/steering_report','/vehicle/brake_report','/vehicle/throttle_report']):
    #for topic, msg, t in bag.read_messages(topics=['/vehicle/steering_report']):
    #print('no {0} - {1} - {2}'.format(topic, t,msg))
    #print('line {0},{1}:{2}'.format(t,topic,msg))
    #print('line {0},{1}'.format(t,topic))

file = 'file.json'
with open(file,'w') as f:
    json.dump(msgList,f)
print('data dumped to {}'.format(file))


bag.close()