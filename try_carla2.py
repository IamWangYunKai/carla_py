# -*- coding: utf-8 -*-
"""
Created on Fri Aug  9 20:54:55 2019
@author: Wang
"""

import random
from time import sleep

import carla
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.basic_agent import BasicAgent

host = "210.32.144.24"
#host = "localhost"
port = 2000
actor_list = []

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
    
    #set_weather(world)
    
    try:
        blueprint_library = world.get_blueprint_library()
        
        # add vehicle
        vehicle = add_vehicle_component(world, blueprint_library)
        # put the vehicle to drive around.
        vehicle.set_autopilot(True)
        
        map = world.get_map()
        
        destination = carla.Location(x=6.07782, y=-105.429, z=-0.035137)
        
        agent = BasicAgent(vehicle)
        agent.set_destination([destination.x, destination.y, destination.z])
        
        dao = GlobalRoutePlannerDAO(map)
        planner = GlobalRoutePlanner(dao)
        planner.setup()
        
        for _ in range(500):
            control = agent.run_step()
            vehicle.apply_control(control)
            
            origin = vehicle.get_location()
            turn_list = planner.abstract_route_plan(origin, destination)
            print('next cmd:',turn_list[0])
            
            sleep(0.01)
        
    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        actor_list = []
        print('done.')
        
if __name__ == '__main__':
    main()