# -*- coding: utf-8 -*-
import os
import random
import pygame
import carla
from auto_ctrl import HUD, World, KeyboardControl
from agents.navigation.basic_agent import BasicAgent

HOST = '127.0.0.1'
PORT = 2000
DATASET_NUM = 10
MAX_FRAMES = 5000 #max frmaes for each dataset
# imgae must be small !!
IMG_LENGTH = 640#320
IMG_WIDTH = 480#240
FOV = 120 # degree
MAX_SPEED = 30 # km/h, slow down for better ctrl !
TIMEOUT = 10.0 # s, connection timeout
CAMERA_TRANS = carla.Transform(
        location = carla.Location(x=1.5, y=0.0, z=2.4),
        rotation = carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))

OUTPUT_PATH = 'OUTPUT'
actor_list = []
world = None

def add_semantic_camera(blueprint_library, vehicle):
    global world, actor_list
    camera_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
    camera_bp.set_attribute('image_size_x', str(IMG_LENGTH))
    camera_bp.set_attribute('image_size_y', str(IMG_WIDTH))
    camera_bp.set_attribute('fov', str(FOV))
    camera_transform = CAMERA_TRANS
    camera = world.world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)
    return camera

def add_rgb_camera(blueprint_library, vehicle):
    global world, actor_list
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(IMG_LENGTH))
    camera_bp.set_attribute('image_size_y', str(IMG_WIDTH))
    camera_bp.set_attribute('fov', str(FOV))
    camera_transform = CAMERA_TRANS
    camera = world.world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)
    return camera

def add_depth_camera(blueprint_library, vehicle):
    global world, actor_list
    camera_bp = blueprint_library.find('sensor.camera.depth')
    camera_bp.set_attribute('image_size_x', str(IMG_LENGTH))
    camera_bp.set_attribute('image_size_y', str(IMG_WIDTH))
    camera_bp.set_attribute('fov', str(FOV))
    camera_transform = CAMERA_TRANS
    camera = world.world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    actor_list.append(camera)
    return camera

def get_rgb_img(image):
    global actor_list, world
    player_transform = world.player.get_transform()
    with open(OUTPUT_PATH + '/location.txt', 'a') as file:
        file.write(str(image.frame_number) + '\t'
                 + str(player_transform.location.x) + '\t'
                 + str(player_transform.location.y) + '\t'
                 + str(player_transform.location.z) + '\t'
                 + str(player_transform.rotation.pitch) + '\t'
                 + str(player_transform.rotation.yaw) + '\t'
                 + str(player_transform.rotation.roll) + '\t'
                 + '\n')
    image.save_to_disk(OUTPUT_PATH + '/%06d' % image.frame_number + '_rgb.png')
  
def get_semantic_img(image):
    image.convert(carla.ColorConverter.CityScapesPalette)
    image.save_to_disk(OUTPUT_PATH + '/%06d' % image.frame_number + '_seg.png')
    
def get_depth_img(image):
    # for each pixel:
    # normalized = (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1)
    # in_meters = 1000 * normalized
    image.convert(carla.ColorConverter.LogarithmicDepth)
    image.save_to_disk(OUTPUT_PATH + '/%06d' % image.frame_number + '_dep.png')

def game_loop():
    global world
    pygame.init()
    pygame.font.init()

    try:
        client = carla.Client(HOST, PORT)
        client.set_timeout(TIMEOUT)

        display = pygame.display.set_mode(
            (IMG_LENGTH, IMG_WIDTH),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        hud = HUD(IMG_LENGTH, IMG_WIDTH)
        world = World(client.get_world(), hud, 'vehicle.bmw.grandtourer')
        controller = KeyboardControl(world, False)
        

        
        # get random starting point and destination
        spawn_points = world.map.get_spawn_points()
        #start_transform = world.player.get_transform()
        start_transform = spawn_points[1]
        world.player.set_transform(start_transform)
        #destination = spawn_points[random.randint(0,len(spawn_points)-1)]
        destination = spawn_points[0]

        agent = BasicAgent(world.player, target_speed=MAX_SPEED)
        agent.set_destination((destination.location.x,
                               destination.location.y,
                               destination.location.z))

        blueprint_library = world.world.get_blueprint_library()
        vehicle = world.player

        rgb_camera = add_rgb_camera(blueprint_library, vehicle)
        depth_camera = add_depth_camera(blueprint_library, vehicle)
        semantic_camera = add_semantic_camera(blueprint_library, vehicle)
        
        #rgb_camera.listen(lambda image: get_rgb_img(image))
        #depth_camera.listen(lambda image: get_depth_img(image))
        #semantic_camera.listen(lambda image: get_semantic_img(image))

        clock = pygame.time.Clock()
        weather = carla.WeatherParameters(
                cloudyness=random.randint(0,80),#0-100
                precipitation=0,#random.randint(0,20),#0-100
                precipitation_deposits=0,#random.randint(0,20),#0-100
                wind_intensity=0,#random.randint(0,50),#0-100
                sun_azimuth_angle=random.randint(30,120),#0-360
                sun_altitude_angle=random.randint(30,90))#-90~90
        
        world.world.set_weather(weather)
         # put vehicle on starting point
        world.player.set_transform(start_transform)

        frames = 0
        while frames < MAX_FRAMES:
            frames += 1
            if controller.parse_events(client, world, clock):
                return

            # as soon as the server is ready continue!
            if not world.world.wait_for_tick(TIMEOUT):
                continue

            world.tick(clock)
            world.render(display)
            pygame.display.flip()
            control = agent.run_step()
            control.manual_gear_shift = False
            world.player.apply_control(control)

    finally:
        rgb_camera.stop()
        depth_camera.stop()
        semantic_camera.stop()
        if world is not None:
            world.destroy()

        pygame.quit()

def main():
    global OUTPUT_PATH
    try:
        for i in range(DATASET_NUM):
            OUTPUT_PATH = 'output' + str(i)
            if not os.path.isdir(OUTPUT_PATH):
                os.makedirs(OUTPUT_PATH)
            game_loop()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

if __name__ == '__main__':
    main()
