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
        add_camera_component(world, blueprint_library, vehicle)
        while True:
            vehicle.set_transform(next_point)
            my_location = next_point.location
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
            """
            for waypoint in waypoints[0:30]:
                world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=False,
                                       color=carla.Color(r=255, g=0, b=0), life_time=1.0,
                                       persistent_lines=True)
            """
            print(len(waypoints))
            next_point = waypoints[random.randint(0,min(len(waypoints)-1, 50))].transform
            #vehicle.set_transform(next_point)
            #my_location = next_point.location
            
            for waypoint in waypoints[5:min(len(waypoints)-1, 50)]:
                box_point = carla.Location(waypoint.transform.location.x,
                                           waypoint.transform.location.y,
                                           waypoint.transform.location.z-0.4
                                           )
                box = carla.BoundingBox(box_point, carla.Vector3D(x=2,y=1.0,z=0.5))
                rotation = carla.Rotation(pitch=waypoint.transform.rotation.pitch, 
                                          yaw=waypoint.transform.rotation.yaw, 
                                          roll=waypoint.transform.rotation.roll)
                world.debug.draw_box(box=box, rotation=rotation, thickness=1.2)
                
            #box = carla.BoundingBox(my_location, carla.Vector3D(x=1,y=1,z=0.01))
            #rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
            #world.debug.draw_box(box=box, rotation=rotation, thickness=1)
            sleep(0.3)
        
    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        actor_list = []
        print('done.')
        
if __name__ == '__main__':
    main()