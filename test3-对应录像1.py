#目的是让car在选择路线后，从起点到达终点，并且在视角中标出路线
#使用的是carla自带的自动驾驶，未涉及算法


import carla 
import time 
import cv2 
import numpy as np 
# 连接到sim 
client = carla.Client('localhost', 2000)
#定义环境/世界，并获得可能的car出现地
world = client.get_world()
spawn_points = world.get_map().get_spawn_points()
#找到car1
vehicle_bp = world.get_blueprint_library().filter('*car1*')

start_point = spawn_points[0]
vehicle = world.try_spawn_actor(vehicle_bp[0], start_point)
#设置RGB相机

# 相机在汽车上的位置
CAMERA_POS_Z = 3 
CAMERA_POS_X = -5 

camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640')
camera_bp.set_attribute('image_size_y', '360')

camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z,x=CAMERA_POS_X))
#在sim中创建相机
camera = world.spawn_actor(camera_bp,camera_init_trans,attach_to=vehicle)

def camera_callback(image,data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

image_w = camera_bp.get_attribute('image_size_x').as_int()
image_h = camera_bp.get_attribute('image_size_y').as_int()

camera_data = {'image': np.zeros((image_h,image_w,4))}
# 打开摄像机的实时流
camera.listen(lambda image: camera_callback(image,camera_data))
#  路线规划
import sys
sys.path.append('C:\CARLA_0.9.14\WindowsNoEditor\PythonAPI\carla') 
from agents.navigation.global_route_planner import GlobalRoutePlanner

point_a = start_point.location 

sampling_resolution = 1
grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)


distance = 0
for loc in spawn_points: 
    cur_route = grp.trace_route(point_a, loc.location)
    if len(cur_route)>distance:
        distance = len(cur_route)
        route = cur_route
#在sim窗口中绘制路线
for waypoint in route:
    world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
        color=carla.Color(r=0, g=0, b=255), life_time=60.0,
        persistent_lines=True)
for waypoint in route:
    
    # 将汽车移动到当前路点
    vehicle.set_transform(waypoint[0].transform)
    # Dispaly with imshow
    cv2.imshow('Fake self-driving',camera_data['image'])
    cv2.waitKey(50)
    
time.sleep(2)
cv2.destroyAllWindows()
camera.stop() 
for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()
for sensor in world.get_actors().filter('*sensor*'):
    sensor.destroy()