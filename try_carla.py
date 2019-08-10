# -*- coding: utf-8 -*-
"""
Created on Fri Aug  9 20:54:55 2019
@author: Wang
"""

import time
import random

import plotly
import plotly.graph_objs as go

import carla
host = "210.32.144.24"
#host = "localhost"
port = 2000
actor_list = []

def show_img(x, y, z, file_name):
    trace = go.Scatter3d(
        x=x,
        y=y,
        z=z,
        mode='markers',
        marker=dict(
            size=3,
            color=z,                # set color to an array/list of desired values
            colorscale='Viridis',   # choose a colorscale
            opacity=0.6,            #不透明度
            showscale =True
        )
    )

    data = [trace]
    layout = go.Layout(
        margin=dict(
            l=0,
            r=0,
            b=0,
            t=0
        )
    )
    fig = go.Figure(data=data, layout=layout)
    plotly.offline.plot(fig, filename=file_name)
    
def deal_lidar_data(data):
    x = []
    y = []
    z = []
    for location in data:
        x.append(location.x)
        y.append(location.y)
        z.append(-location.z)
    file_name = 'pcd_' + str(data.frame_number) + '.html'
    show_img(x, y, z, file_name)

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
      
def add_lidar_component(world, blueprint_library, vehicle):
    global actor_list
    # add lidar
    lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('channels', '32')
    lidar_bp.set_attribute('range', '1000') #1000 cm
    lidar_bp.set_attribute('points_per_second', '5600000')
    lidar_bp.set_attribute('rotation_frequency', '200.0')
    lidar_bp.set_attribute('upper_fov', '60.0')
    lidar_bp.set_attribute('lower_fov', '-60.0')
    #lidar_bp.set_attribute('sensor_tick', '0.0')
    
    lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.5))
    lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
    actor_list.append(lidar)
    print('created %s' % lidar.type_id)
    #lidar.listen(lambda data: print(list(data.raw_data)))
    lidar.listen(lambda data: deal_lidar_data(data))
    return lidar
        
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
        
        add_camera_component(world, blueprint_library, vehicle)
        add_lidar_component(world, blueprint_library, vehicle)
        
        time.sleep(1.0)
        
    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        actor_list = []
        print('done.')
        
if __name__ == '__main__':
    main()