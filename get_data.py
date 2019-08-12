# -*- coding: utf-8 -*-
"""
@brief: Generate visual navigation data for CARLA 0.9.5
@author: Wang Yunkai
@e-mail: wangyunkai.zju@gmail.com
@data: 2019.8.12
"""

import math
import random
import numpy as np
from time import sleep
import matplotlib.pyplot as plt
from matplotlib.image import imsave

import carla
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.global_route_planner import GlobalRoutePlanner
#from agents.navigation.basic_agent import BasicAgent

# CARLA Server Setting
host = "localhost"
port = 2000
TIME_OUT = 5.0 # seconds

# global variable
actor_list = []
flag = False # get img ?
semantic_flag = False # get semantic img ?
frame = 0 # frame number
ref_img = None
senmantic_img = None
world = None

# const variable
IMG_LENGTH = 800
IMG_WIDTH = 600
OUTPUT_PATH = 'output/'

OBSTACLE = np.array((0, 0, 255, 255))
ROAD = np.array((255, 0, 0, 255))
NOTHING = np.array((0, 0, 0, 255))
DEBUG = np.array((0, 255, 0, 255))

def add_vehicle(blueprint_library):
    global world
    bp = random.choice(blueprint_library.filter('vehicle.bmw.grandtourer'))
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
    transform = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(bp, transform)
    actor_list.append(vehicle)
    return vehicle

def add_semantic_camera(blueprint_library, vehicle):
    global world, actor_list
    camera_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
    camera_bp.set_attribute('image_size_x', str(IMG_LENGTH))
    camera_bp.set_attribute('image_size_y', str(IMG_WIDTH))
    camera_bp.set_attribute('fov', '90') #视角度数
    camera_bp.set_attribute('sensor_tick', '0.3')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)
    return camera

def add_camera(blueprint_library, vehicle):
    global world, actor_list
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(IMG_LENGTH))
    camera_bp.set_attribute('image_size_y', str(IMG_WIDTH))
    camera_bp.set_attribute('fov', '90') #视角度数
    camera_bp.set_attribute('sensor_tick', '0.3')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)
    return camera

def set_weather():
    global world
    weather = carla.WeatherParameters( 
                cloudyness=0,
                precipitation=0,
                precipitation_deposits=0,
                wind_intensity=0,
                sun_azimuth_angle=90,
                sun_altitude_angle=90) 
    world.set_weather(weather)
    return weather

def get_raw_img(image):
    global flag, frame
    # set frame number
    frame = image.frame_number
    flag = True
    image.save_to_disk(OUTPUT_PATH + '%06d' % frame + '_raw.png')
  
def get_semantic_img(image):
    global semantic_flag, ref_img, senmantic_img
    semantic_flag = True
    image.convert(carla.ColorConverter.CityScapesPalette)
    # save to global data
    senmantic_img = np.resize(np.array(image.raw_data), (IMG_WIDTH,IMG_LENGTH,4))
    
def get_nav_img(image):
    global flag, frame, ref_img
    # save to global data
    ref_img = np.resize(np.array(image.raw_data), (IMG_WIDTH,IMG_LENGTH,4))
    flag = True
    image.save_to_disk(OUTPUT_PATH + '%06d' % frame + '_nav.png')
    
def deal_semantic_img():
    global frame, senmantic_img
    convert_img = senmantic_img
    for i in range(IMG_WIDTH):
        for j in range(IMG_LENGTH):
            pixel = convert_img[i][j]
            if pixel[0] == 70: #Building
                convert_img[i][j] = OBSTACLE
            elif pixel[0] == 190: #Fence
                convert_img[i][j] = OBSTACLE
            elif pixel[0] == 160: #Other
                convert_img[i][j] = OBSTACLE
            elif pixel[0] == 220: #Pedestrian
                convert_img[i][j] = OBSTACLE
            elif pixel[0] == 153: #Pole
                convert_img[i][j] = OBSTACLE
            elif pixel[0] == 50: #Road line
                convert_img[i][j] = ROAD
            elif pixel[0] == 128: #Road
                convert_img[i][j] = ROAD
            elif pixel[0] == 244: #Sidewalk
                convert_img[i][j] = ROAD
            elif pixel[0] == 35: #Vegetation
                convert_img[i][j] = OBSTACLE
            elif pixel[0] == 232: #may be Sidewalk
                convert_img[i][j] = OBSTACLE
            elif pixel[0] == 156: #may be build
                convert_img[i][j] = OBSTACLE
            elif pixel[0] == 142: #Car
                convert_img[i][j] = OBSTACLE
            elif pixel[1] == 220: #Traffic sign
                convert_img[i][j] = OBSTACLE

            ref_pixel = ref_img[i][j]
            if ref_pixel[2] > 160 and ref_pixel[0] <30 and ref_pixel[1] <30:
                convert_img[i][j] = DEBUG
    imsave(OUTPUT_PATH + '%06d' % frame + '_seg.png', convert_img)

def get_instruction(waypoints):
    global frame
    x = []
    y = []
    theta = math.atan2((waypoints[3].transform.location.y - waypoints[0].transform.location.y),
                       (waypoints[3].transform.location.x - waypoints[0].transform.location.x))
    for i in range(min(len(waypoints)-1, 50)):
        _x = waypoints[i].transform.location.x - waypoints[0].transform.location.x
        _y = waypoints[i].transform.location.y - waypoints[0].transform.location.y
        
        new_theta = math.pi/2-theta

        x_ = _x*math.cos(new_theta) - _y*math.sin(new_theta)
        y_ = _y*math.cos(new_theta) + _x*math.sin(new_theta)
        
        x.append(-x_)
        y.append(y_)
    
    scale = 20
    fig = plt.figure(figsize=(IMG_LENGTH/100,IMG_WIDTH/100))
    plt.xlim(-scale, scale)
    plt.ylim(0, scale)
    plt.axis('off')
    plt.plot(x,y,"r-",linewidth=50)
    fig.savefig(OUTPUT_PATH + '%06d' % frame + '_ins.png', bbox_inches='tight', dpi=400)
    plt.close(fig)

def draw_lane(waypoints):
    global world
    # draw lane
    for waypoint in waypoints[0:min(len(waypoints)-1, 20)]:
        box_point = carla.Location(waypoint.transform.location.x,
                                   waypoint.transform.location.y,
                                   waypoint.transform.location.z-0.4)
        box = carla.BoundingBox(box_point, carla.Vector3D(x=2,y=0.1,z=0.4))
        rotation = carla.Rotation(pitch=waypoint.transform.rotation.pitch, 
                                  yaw=waypoint.transform.rotation.yaw, 
                                  roll=waypoint.transform.rotation.roll)
        world.debug.draw_box(box=box, rotation=rotation, thickness=1.2, life_time=0)
    # wait for draw
    sleep(0.3)
      
def main():
    global world, actor_list, flag, semantic_flag, frame
    client = carla.Client(host, port)
    client.set_timeout(TIME_OUT)
    try:
        world = client.get_world()
    except:
        print("ERROR: Cannot get world !")
        import sys
        sys.exit()
    
    set_weather()
    
    try:
        blueprint_library = world.get_blueprint_library()
        
        # add vehicle
        vehicle = add_vehicle(blueprint_library)
        # put the vehicle to drive around.
        #vehicle.set_autopilot(True)
        
        map = world.get_map()
        spawn_points = map.get_spawn_points()
        destination = spawn_points[random.randint(0,len(spawn_points)-1)].location
        #agent = BasicAgent(vehicle, target_speed=20)
        #agent.set_destination((destination.x,
        #                       destination.y,
        #                       destination.z))
        dao = GlobalRoutePlannerDAO(map)
        planner = GlobalRoutePlanner(dao)
        planner.setup()
        
        vehicle.set_simulate_physics(False)
        
        my_location = vehicle.get_location()
        trace_list = planner.trace_route(my_location, destination)
        waypoints = []
        for (waypoint, road_option) in trace_list:
            waypoints.append(waypoint)
        next_point = waypoints[0].transform
        
        camera = add_camera(blueprint_library, vehicle)
        semantic_camera = add_semantic_camera(blueprint_library, vehicle)
        
        while True:
            vehicle.set_transform(next_point)
            my_location = next_point.location
            #my_location = vehicle.get_location()
            me2destination = my_location.distance(destination)
            # too close to the destination, choose another one
            if me2destination < 50 :
                destination = spawn_points[random.randint(0,len(spawn_points)-1)].location
                #agent.set_destination((destination.x,destination.y,destination.z))
                print("destination change !!!")
                
            # get planed path
            trace_list = planner.trace_route(my_location, destination)
            waypoints = []
            for (waypoint, road_option) in trace_list:
                waypoints.append(waypoint)
            
            # get raw image
            camera.listen(lambda image: get_raw_img(image))
            while not flag:
                sleep(0.001)
            camera.stop()
            flag = False
            
            # get semantic imgae
            semantic_camera.listen(lambda image: get_semantic_img(image))
            while not semantic_flag:
                sleep(0.001)
            semantic_camera.stop()
            semantic_flag = False
            
            get_instruction(waypoints)
            
            draw_lane(waypoints)
            
            # get ground truth
            camera.listen(lambda image: get_nav_img(image))
            while not flag:
                sleep(0.001)
            camera.stop()
            flag = False
            
            deal_semantic_img()
            
            # wait debug infomation disappear
            sleep(1.0)
            
            # choose a new point for vehicle
            next_point = waypoints[random.randint(0,min(len(waypoints)-1, 50))].transform
        
    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        actor_list = []
        print('done.')
        
if __name__ == '__main__':
    main()