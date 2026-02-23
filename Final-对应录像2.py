#目的是让car通过摄像头实时判断路况，进行制动
#使用的算法较为简单，为基本三角关系函数

import carla 
import cv2 
import time 
import numpy as np 
import math
import random
import sys
sys.path.append('C:\CARLA_0.9.14\WindowsNoEditor\PythonAPI\carla') # tweak to where you put carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

# 连接到sim
client = carla.Client('localhost', 2000)

# 定义速度常数
PREFERRED_SPEED = 70 # 定义最高速度
SPEED_THRESHOLD = 2 #定义何时接近所需速度，从而降低

# 最大转向角
MAX_STEER_DEGREES = 40
# 这是Car转向不足输入=1.0时的最大实际角度
STEERING_CONVERSION = 75

#相机支架在汽车上的偏移
CAMERA_POS_Z = 3 
CAMERA_POS_X = -5 

#在图像中显示文本
font = cv2.FONT_HERSHEY_SIMPLEX
# org - 定义行以在屏幕上显示遥测值
org = (30, 30) # 显示当前速度
org2 = (30, 50) # 用于未来的转向角
org3 = (30, 70) # 
org4 = (30, 90) # 
org3 = (30, 110) # 用于未来遥测输出的线路
fontScale = 0.5
# 白色
color = (255, 255, 255)
# 线条厚度为2像素
thickness = 1

# 摄像头监听实用功能
def camera_callback(image,data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

# 保持速度功能
def maintain_speed(s):
    ''' 
    这是一个非常简单的功能，可以保持所需的速度 s和arg是当前实际速度
    '''
    if s >= PREFERRED_SPEED:
        return 0
    elif s < PREFERRED_SPEED - SPEED_THRESHOLD:
        return 0.9 
    else:
        return 0.4 


    # 获取汽车和目标航路点之间的角度的功能
def get_angle(car,wp):
    '''
    该函数返回汽车方向之间的度数以及指向选定航路点的方向
    '''
    vehicle_pos = car.get_transform()
    car_x = vehicle_pos.location.x
    car_y = vehicle_pos.location.y
    wp_x = wp.transform.location.x
    wp_y = wp.transform.location.y
    
    # 矢量到航路点
    x = (wp_x - car_x)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
    y = (wp_y - car_y)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
    
    #汽车矢量
    car_vector = vehicle_pos.get_forward_vector()
    degrees = math.degrees(np.arctan2(y, x) - np.arctan2(car_vector.y, car_vector.x))
    #返回接近360度的值时，对预测角度进行额外检查
    if degrees<-300:
        degrees = degrees + 360
    elif degrees > 300:
        degrees = degrees - 360
    return degrees

def get_proper_angle(car,wp_idx,rte):
    '''
    确保我们没有跳过下一个航路点，这样我们就可以避开试图折返的汽车
    '''
    # 创建一个从当前开始到下5个路点的角度列表
    next_angle_list = []
    for i in range(10):
        if wp_idx + i*3 <len(rte)-1:
            next_angle_list.append(get_angle(car,rte[wp_idx + i*3][0]))
    idx = 0
    while idx<len(next_angle_list)-2 and abs(next_angle_list[idx])>40:
        idx +=1
    return wp_idx+idx*3,next_angle_list[idx]  

def draw_route(wp, route,seconds=3.0):
    #在sim窗口中绘制接下来的几个点路线
    # 进入汽车的摄像头
    if len(route)-wp <25: # 距离终点25点以内的路线为红色
        draw_colour = carla.Color(r=255, g=0, b=0)
    else:
        draw_colour = carla.Color(r=0, g=0, b=255)
    for i in range(10):
        if wp+i<len(route)-2:
            world.debug.draw_string(route[wp+i][0].transform.location, '^', draw_shadow=False,
                color=draw_colour, life_time=seconds,
                persistent_lines=True)
    return None


def select_random_route(position,locs):
    '''
    为汽车/车辆重新运行随机路线,在可能的位置列表中,locs其中距离大于100个路点
    '''    
    point_a = position.location #我们从汽车所在地或最后一个航路点开始
    sampling_resolution = 1
    grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
    # 现在让我们选择最长的路线
    min_distance = 100
    result_route = None
    route_list = []
    for loc in locs:                            
        cur_route = grp.trace_route(point_a, loc.location)
        if len(cur_route) > min_distance:
            route_list.append(cur_route)
    result_route = random.choice(route_list)
    end_of_route = result_route[-1][0].transform 
    return result_route,end_of_route

world = client.get_world()
spawn_points = world.get_map().get_spawn_points()
#找到车辆
vehicle_bp = world.get_blueprint_library().filter('*mini*')
start_point = spawn_points[0]
vehicle = world.try_spawn_actor(vehicle_bp[0], start_point)



camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640') 
camera_bp.set_attribute('image_size_y', '360')
camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z,x=CAMERA_POS_X))
# 这将在sim中创建相机
camera = world.spawn_actor(camera_bp,camera_init_trans,attach_to=vehicle)
image_w = camera_bp.get_attribute('image_size_x').as_int()
image_h = camera_bp.get_attribute('image_size_y').as_int()
camera_data = {'image': np.zeros((image_h,image_w,4))}
# 打开摄像机的实时流
camera.listen(lambda image: camera_callback(image,camera_data))
cv2.namedWindow('RGB Camera',cv2.WINDOW_AUTOSIZE)
cv2.imshow('RGB Camera',camera_data['image'])


# 主要算法
start_route = start_point
quit = False
while True:
# 获取汽车的随机路线
    route,start_route = select_random_route(start_route,spawn_points)
    curr_wp = 5 # 我们将跟踪路线中的路点，并在接近当前路点时切换到下一个路点
    predicted_angle = 0
    PREFERRED_SPEED = 40 # 在新路线开始时设置速度
    
    while curr_wp<len(route)-1:
        world.tick()
        draw_route(curr_wp, route,1)
        if cv2.waitKey(1) == ord('q'):
            quit = True
            vehicle.apply_control(carla.VehicleControl(throttle=0,steer=0,brake=1))
            break
        image = camera_data['image']
        if curr_wp >=len(route)-10: # 距离终点不到10点，路线就完成了
            PREFERRED_SPEED = 0 # 完成一条路线后将速度调整为0
            vehicle.apply_control(carla.VehicleControl(throttle=0,steer=0,brake=1))
            break
        while curr_wp<len(route)-2 and vehicle.get_transform().location.distance(route[curr_wp][0].transform.location)<5:
            curr_wp +=1 #如果距离太近，请转到下一个wp
        curr_wp, predicted_angle = get_proper_angle(vehicle,curr_wp,route)
        image = cv2.putText(image, 'Steering angle: '+str(round(predicted_angle,1)), org, font, fontScale, color, thickness, cv2.LINE_AA)
        v = vehicle.get_velocity()
        speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),0)
        image = cv2.putText(image, 'Speed: '+str(int(speed)), org3, font, fontScale, color, thickness, cv2.LINE_AA)
        image = cv2.putText(image, 'Next waypoint: '+str(curr_wp), org2, font, fontScale, color, thickness, cv2.LINE_AA)
        estimated_throttle = maintain_speed(speed)

        steer_input = predicted_angle
        # 将转向限制在最大角度，例如40度
        if predicted_angle<-MAX_STEER_DEGREES:
            steer_input = -MAX_STEER_DEGREES
        elif predicted_angle>MAX_STEER_DEGREES:
            steer_input = MAX_STEER_DEGREES
       #应用控制功能的从度到-1到+1的转换输入在下面的转向输入中应用
        vehicle.apply_control(carla.VehicleControl(throttle=estimated_throttle, steer=steer_input/STEERING_CONVERSION))
        cv2.imshow('RGB Camera',image)
    if quit:
        break

#clean up
cv2.destroyAllWindows()
camera.stop()
for sensor in world.get_actors().filter('*sensor*'):
    sensor.destroy()
for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()