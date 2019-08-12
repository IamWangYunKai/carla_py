# -*- coding: utf-8 -*-
"""
Created on Mon Aug 12 10:00:40 2019
@author: Wang
"""

import time
import random
import numpy as np
from matplotlib.image import imsave

import carla
#host = "210.32.144.24"
host = "localhost"
port = 2000
actor_list = []

OBSTACLE = np.array((0, 0, 255, 255))
ROAD = np.array((255, 0, 0, 255))
NOTHING = np.array((0, 0, 0, 255))
DEBUG = np.array((0, 255, 0, 255))

def set_weather(world):
        # change weather
    weather = carla.WeatherParameters( 
                cloudyness=30.0, #云量
                precipitation=10, #降水
                precipitation_deposits=10, #降水沉积物
                wind_intensity=10, #风力强度
                sun_azimuth_angle=20, #太阳方位角
                sun_altitude_angle=80.0 #太阳高度角
            ) 
    
    world.set_weather(weather)
    return weather
    
def add_vehicle_component(world, blueprint_library):
    bp = random.choice(blueprint_library.filter('vehicle'))
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
        
    transform = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(bp, transform)
    actor_list.append(vehicle)
    print('created %s' % vehicle.type_id)
    return vehicle
        
def add_camera_component(world, blueprint_library, vehicle):
    global actor_list
    # add camera
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    # Modify the attributes of the blueprint to set image resolution and field of view.
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90') #视角度数
    # Set the time in seconds between sensor captures
    #camera_bp.set_attribute('sensor_tick', '0.0')
    
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)
    print('created %s' % camera.type_id)
    camera.listen(lambda image: image.save_to_disk('_output/%06d.png' % image.frame_number))
    return camera

def save_semantic_img(image, frame_number):
    image.convert(carla.ColorConverter.CityScapesPalette)
    convert_img = np.resize(np.array(image.raw_data), (600,800,4))
    for i in range(600):
        for j in range(800):
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
                convert_img[i][j] = NOTHING

    imsave('seg_output/%06d.png' % image.frame_number, convert_img)
    
    #image.save_to_disk('output/%06d.png' % frame_number)
    
def add_semantic_camera_component(world, blueprint_library, vehicle):
    global actor_list
    # add camera
    camera_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
    # Modify the attributes of the blueprint to set image resolution and field of view.
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90') #视角度数
    # Set the time in seconds between sensor captures
    #camera_bp.set_attribute('sensor_tick', '0.0')
    
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)
    print('created %s' % camera.type_id)
    camera.listen(lambda image: save_semantic_img(image, image.frame_number))
    return camera
        
def main():
    global actor_list
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    
    try:
        world = client.get_world()
    except:
        print("ERROR: Cannot get world !")
        import sys
        sys.exit()
    
    set_weather(world)
    
    try:
        blueprint_library = world.get_blueprint_library()
        
        # add vehicle
        vehicle = add_vehicle_component(world, blueprint_library)
        # put the vehicle to drive around.
        vehicle.set_autopilot(True)
        
        #add_camera_component(world, blueprint_library, vehicle)
        add_semantic_camera_component(world, blueprint_library, vehicle)
        
        time.sleep(9999)
        
    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        actor_list = []
        print('done.')
        
if __name__ == '__main__':
    main()