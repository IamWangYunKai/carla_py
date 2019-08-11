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

#host = "210.32.144.24"
host = "localhost"
port = 2000
actor_list = []

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
        
        while True:
            #my_location = vehicle.get_location()
            me2destination = my_location.distance(destination)
            if me2destination < 20 :
                destination = spawn_points[random.randint(0,len(spawn_points)-1)].location
                #agent.set_destination((destination.x,destination.y,destination.z))
                print("destination change !!!")
                
            trace_list = planner.trace_route(my_location, destination)
            waypoints = []
            for (waypoint, road_option) in trace_list:
                waypoints.append(waypoint)
            
            for waypoint in waypoints[0:30]:
                world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                       color=carla.Color(r=255, g=0, b=0), life_time=1.0,
                                       persistent_lines=True)
            
            next_point = waypoints[random.randint(0,min(len(waypoints)-1, 50))].transform
            vehicle.set_transform(next_point)
            my_location = next_point.location
            #sleep(0.1)

        """
        while True:
            control = agent.run_step()
            #vehicle.apply_control(control)
            
            my_location = vehicle.get_location()
            me2destination = my_location.distance(destination)
            if me2destination < 10 :
                destination = spawn_points[random.randint(0,len(spawn_points)-1)].location
                agent.set_destination((destination.x,
                                       destination.y,
                                       destination.z))
                print("destination change !!!")

            trace_list = planner.trace_route(my_location, destination)
            if len(trace_list) < 4:
                continue
            for (waypoint, road_option) in [trace_list[-1], trace_list[-2], trace_list[-3], trace_list[-4]]:
                world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                                   color=carla.Color(r=255, g=0, b=0), life_time=1.0,
                                                   persistent_lines=True)
                
            (waypoint, road_option) = trace_list[-1]
            vehicle.set_transform(waypoint.transform)
            sleep(0.5)

            waypoints = agent._local_planner._waypoint_buffer
            for waypoint in [waypoints[-1], waypoints[-2], waypoints[-3], waypoints[-4]]:
                world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                        color=carla.Color(r=255, g=0, b=0), life_time=1.0,
                                        persistent_lines=True)
            
            """
                
        """
        # Retrieve the closest waypoint.
        waypoint = map.get_waypoint(vehicle.get_location())
        
        # Disable physics, in this example we're just teleporting the vehicle.
        vehicle.set_simulate_physics(False)
        
        while True:
            # Find next waypoint 2 meters ahead.
            waypoint = map.get_waypoint(vehicle.get_location())
            waypoints = waypoint.next(1.0)
            print(len(waypoints))
            for waypoint in waypoints:
                world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                        color=carla.Color(r=255, g=0, b=0), life_time=1.0,
                                        persistent_lines=True)
            # Teleport the vehicle.
            vehicle.set_transform(waypoint.transform)
            sleep(1)
        """
        
    finally:
        pass
        """
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        actor_list = []
        print('done.')
        """
        
if __name__ == '__main__':
    main()