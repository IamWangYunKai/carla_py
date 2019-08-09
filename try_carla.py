# -*- coding: utf-8 -*-
"""
Created on Fri Aug  9 20:54:55 2019
@author: Wang
"""

import sys
import time
import random

import carla
#host = "210.32.144.24"
host = "localhost"
port = 2000

client = carla.Client(host, port)
client.set_timeout(10.0)

try:
    world = client.get_world()
except:
    print("ERROR: Cannot get world !")
    sys.exit()
    
# change weather
weather = carla.WeatherParameters( 
            cloudyness=30.0, #云量
            precipitation=10, #降水
            precipitation_deposits=10, #降水沉积物
            wind_intensity=10, #风力强度
            sun_azimuth_angle=20, #太阳方位角
            sun_altitude_angle=60.0 #太阳高度角
        ) 

world.set_weather(weather)

actor_list = []
frame = 0
try:
    blueprint_library = world.get_blueprint_library()
    bp = random.choice(blueprint_library.filter('vehicle'))
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
    
    transform = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(bp, transform)
    
    # put the vehicle to drive around.
    vehicle.set_autopilot(True)
    actor_list.append(vehicle)
    print('created %s' % vehicle.type_id)
    
    # add camera
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    # Modify the attributes of the blueprint to set image resolution and field of view.
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90') #视角度数
    # Set the time in seconds between sensor captures
    camera_bp.set_attribute('sensor_tick', '0.1')
    
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)
    print('created %s' % camera.type_id)

    camera.listen(lambda image: image.save_to_disk('output/%06d.png' % image.frame_number))
    frame += 1
    
    time.sleep(5)
    
finally:
    print('destroying actors')
    for actor in actor_list:
        actor.destroy()
    print('done.')