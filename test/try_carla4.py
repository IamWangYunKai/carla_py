# -*- coding: utf-8 -*-
"""
Created on Fri Aug  9 20:54:55 2019
@author: Wang
"""

import math
import random
from time import sleep
import matplotlib.pyplot as plt

import carla
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.basic_agent import BasicAgent

#host = "210.32.144.24"
host = "localhost"
port = 2000
actor_list = []

def add_camera_component(world, blueprint_library, vehicle):
    global actor_list
    # add camera
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    # Modify the attributes of the blueprint to set image resolution and field of view.
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90') #视角度数
    # Set the time in seconds between sensor captures
    camera_bp.set_attribute('sensor_tick', '0.3')
    
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)
    print('created %s' % camera.type_id)
    camera.listen(lambda image: image.save_to_disk('_output/%06d.png' % image.frame_number))
    return camera

def set_weather(world):
        # change weather
    weather = carla.WeatherParameters( 
                cloudyness=20.0, #云量
                precipitation=0, #降水
                precipitation_deposits=0, #降水沉积物
                wind_intensity=10, #风力强度
                sun_azimuth_angle=80, #太阳方位角
                sun_altitude_angle=80.0 #太阳高度角
            ) 
    
    world.set_weather(weather)
    return weather
    
def add_vehicle_component(world, blueprint_library):
    bp = random.choice(blueprint_library.filter('vehicle.bmw.grandtourer'))
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)
        
    transform = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(bp, transform)
    actor_list.append(vehicle)
    print('created %s' % vehicle.type_id)
    return vehicle
        
def main():
    global actor_list
    client = carla.Client(host, port)
    client.set_timeout(5.0)
    
    try:
        world = client.get_world()
    except:
        print("ERROR: Cannot get world !")
        import sys
        sys.exit()
    
    set_weather(world)
    #carla.TrafficLight.set_green_time(float(999))
    #carla.TrafficLight.set_red_time(0)
    
    try:
        blueprint_library = world.get_blueprint_library()
        
        # add vehicle
        vehicle = add_vehicle_component(world, blueprint_library)
        # put the vehicle to drive around.
        #vehicle.set_autopilot(True)
        
        map = world.get_map()
        spawn_points = map.get_spawn_points()
        destination = spawn_points[random.randint(0,len(spawn_points)-1)].location
        
        agent = BasicAgent(vehicle, target_speed=20)
        agent.set_destination((destination.x,
                               destination.y,
                               destination.z))
        
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
        
        #add_camera_component(world, blueprint_library, vehicle)
        
        while True:
            vehicle.set_transform(next_point)
            my_location = next_point.location
            #my_location = vehicle.get_location()
            me2destination = my_location.distance(destination)
            if me2destination < 50 :
                destination = spawn_points[random.randint(0,len(spawn_points)-1)].location
                #agent.set_destination((destination.x,destination.y,destination.z))
                print("destination change !!!")
                
            trace_list = planner.trace_route(my_location, destination)
            waypoints = []
            for (waypoint, road_option) in trace_list:
                waypoints.append(waypoint)

            print(len(waypoints))
            if len(waypoints) < 5 :
                print('my_location:' , my_location.x, my_location.y, my_location.z)
                print('destination:', destination.x, destination.y, destination.z)
                print('me2destination:', me2destination)
                
            next_point = waypoints[random.randint(0,min(len(waypoints)-1, 50))].transform
                
            x = []
            y = []
            #theta = waypoints[0].transform.rotation.roll
            theta = math.atan2((waypoints[3].transform.location.y - waypoints[0].transform.location.y),
                               (waypoints[3].transform.location.x - waypoints[0].transform.location.x))
            for i in range(min(len(waypoints)-1, 50)):
                _x = waypoints[i].transform.location.x - waypoints[0].transform.location.x
                _y = waypoints[i].transform.location.y - waypoints[0].transform.location.y
                
                new_theta = math.pi/2-theta

                x_ = _x*math.cos(new_theta) - _y*math.sin(new_theta)
                y_ = _y*math.cos(new_theta) + _x*math.sin(new_theta)
                
                x.append(x_)
                y.append(y_)
            
            scale = 10
            plt.figure(figsize=(8,4))
            plt.xlim(-scale, scale)
            plt.ylim(0, scale)
            plt.plot(x,y,"r-",linewidth=50)
            plt.show()
            sleep(0.3)
        
    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        actor_list = []
        print('done.')
        
if __name__ == '__main__':
    main()