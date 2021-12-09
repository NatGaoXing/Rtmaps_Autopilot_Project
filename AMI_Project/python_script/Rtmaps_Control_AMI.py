import numpy as np
import rtmaps.types
import rtmaps.core as rt
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent

import math
import glob
import os
import sys

from queue import Queue
from queue import Empty

from openpyxl import Workbook, load_workbook

CARLA_PYTHON_DIRECTORY = "C:/Users/Nicolas/Documents/UTAC Local/CARLA 0_9_11/PythonAPI"

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    sys.path.append(glob.glob(CARLA_PYTHON_DIRECTORY + '/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

try:
    import carla
except ImportError:
    raise RuntimeError('Cannot import carla.')

from carla import VehicleLightState as vls

from Manual_Control_AMI import *
# from Record_Data_AMI import *


def sensor_callback(data, queue):
    queue.put(data)


def getPose(world):
    t = world.player.get_transform()
    v = world.player.get_velocity()
    speedKmh = (3.6 * math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2))

    roll = np.radians(t.rotation.roll)
    pitch = np.radians(-t.rotation.pitch)
    yaw = t.rotation.yaw

    return speedKmh, world.imu_sensor.accelerometer, world.imu_sensor.gyroscope, (
        world.gnss_sensor.lat, world.gnss_sensor.lon, world.gnss_sensor.alt), roll, pitch, yaw, world.gnss_sensor.ts


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)

        self.HOST = '127.0.0.1'
        self.PORT = 2000

        self.WIDTH = 640
        self.HEIGHT = 480
        self.world = None

        self.gamma = 2.2
        self.rolename = 'hero'
        self.filter = 'vehicle.bmw.i*'

        self.walkers_list = []
        self.all_id = []
        self.vehicles_list = []

        self.isFirst = True

        self.client = carla.Client(self.HOST, self.PORT)
        self.client.set_timeout(2.0)

        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode((self.WIDTH, self.HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.display.fill((0, 0, 0))
        pygame.display.flip()

        self.hud = HUD(self.WIDTH, self.HEIGHT)

        self.controller = None
        self.clock = pygame.time.Clock()
        self.image_queue = Queue()
        self.lidar_queue = Queue()
        self.cameraFront = None
        self.lidar = None

        self.traffic_manager = None
        self.all_actors = None

    def Dynamic(self):
        self.add_property("HOST", '127.0.0.1')
        self.add_property("PORT", 2000)
        self.add_property("WIDTH", 1280)
        self.add_property("HEIGHT", 720)
        self.add_property("Gamma", 2.2)
        self.add_property("Rolename", "hero")
        self.add_property("Filter", 'vehicle.bwm.i*')
        self.add_property("TM_PORT", 8000)

        self.add_property("UseSimulationTimeStamp", True)
        self.add_property("WalkerNumberToSpawn", 0)
        self.add_property("VehiclesNumberToSpawn", 0)

        self.add_property("AutoSpawnVehicle", False)
        if not self.properties["AutoSpawnVehicle"].data:
            self.add_property("CAR_LOCATION_X", -8.31)
            self.add_property("CAR_LOCATION_Y", 25.91)
            self.add_property("CAR_LOCATION_Z", 2.0)
            self.add_property("CAR_LOCATION_RX", 0.0)
            self.add_property("CAR_LOCATION_RY", 0.0)
            self.add_property("CAR_LOCATION_RZ", 78.62)

        self.add_property("lidarEnabled", True)
        if self.properties["lidarEnabled"].data:
            self.add_property("LIDAR_MAX_RANGE_IN_M", 50.0)
            self.add_property("LIDAR_NB_LAYERS", 16)
            self.add_property("LIDAR_ROTATION_FREQUENCY_HZ", 20)
            self.add_property("LIDAR_UPPER_FOV_IN_DEG", 15)
            self.add_property("LIDAR_LOWER_FOV_IN_DEG", -15)
            self.add_property("LIDAR_NOISE_IN_M", 0.02)
            self.add_property("LIDAR_CAR_LOCATION_X", 1.0)
            self.add_property("LIDAR_CAR_LOCATION_Y", 0.0)
            self.add_property("LIDAR_CAR_LOCATION_Z", 1.8)
            self.add_property("LIDAR_CAR_LOCATION_RX", 0.0)
            self.add_property("LIDAR_CAR_LOCATION_RY", 0.0)
            self.add_property("LIDAR_CAR_LOCATION_RZ", 0.0)

        self.add_property("cameraEnabled", True)
        if self.properties["cameraEnabled"].data:
            self.add_property("CAMERA_IMAGE_WIDTH", 640)
            self.add_property("CAMERA_IMAGE_HEIGHT", 480)
            self.add_property("CAMERA_IMAGE_FOV", 69.0)  # realsense h fov

            self.add_property("CAMERA_LOCATION_X", 1.5)
            self.add_property("CAMERA_LOCATION_Y", -0.1)
            self.add_property("CAMERA_LOCATION_Z", 1.8)
            self.add_property("CAMERA_LOCATION_RX", 0.0)
            self.add_property("CAMERA_LOCATION_RY", -15.0)
            self.add_property("CAMERA_LOCATION_RZ", 0.0)

        self.add_input("in", rtmaps.types.ANY)

        self.add_output("state", rtmaps.types.FLOAT64)
        self.add_output("speedKmh", rtmaps.types.FLOAT64)
        self.add_output("accXYZ", rtmaps.types.FLOAT64)
        self.add_output("gyroXYZ", rtmaps.types.FLOAT64)
        self.add_output("imuLatLongAlt", rtmaps.types.FLOAT64)
        self.add_output("rollPitchYaw", rtmaps.types.FLOAT64)
        self.add_output("ptCloud", rtmaps.types.FLOAT64, buffer_size=100000)
        self.add_output("imageCamera", rtmaps.types.AUTO)

    def Birth(self):
        pass

    def Core(self):
        if self.isFirst:
            self.isFirst = False

            if True:
                argparser = argparse.ArgumentParser(description='CARLA Manual Control Client')
                args = argparser.parse_args()
                args.gamma = self.gamma
                args.rolename = self.rolename
                args.filter = self.filter

                carPose = None
                if not self.properties["AutoSpawnVehicle"].data:
                    carPose = carla.Transform(carla.Location(x=self.properties["CAR_LOCATION_X"].data,
                                                             y=self.properties["CAR_LOCATION_Y"].data,
                                                             z=self.properties["CAR_LOCATION_Z"].data),
                                              carla.Rotation(pitch=self.properties["CAR_LOCATION_RY"].data,
                                                             roll=self.properties["CAR_LOCATION_RX"].data,
                                                             yaw=self.properties["CAR_LOCATION_RZ"].data))

                self.world = World(self.client.get_world(), self.hud, carPose, args)

                self.controller = KeyboardControl(self.world, False)

                if self.properties["WalkerNumberToSpawn"].data > 0:  # spawn walker if needed
                    self.spawnWalker(self.properties["walkerNumberToSpawn"].data)

                if self.properties["VehiclesNumberToSpawn"].data > 0:  # spawn vehicles if needed
                    self.spawnVehicles(self.properties["vehiclesNumberToSpawn"].data)

                # spawn cameraEnabled
                if self.properties["cameraEnabled"].data:
                    camera_bp = self.world.world.get_blueprint_library().filter("sensor.camera.rgb")[0]
                    camera_bp.set_attribute("image_size_x", str(self.properties["CAMERA_IMAGE_WIDTH"].data))
                    camera_bp.set_attribute("image_size_y", str(self.properties["CAMERA_IMAGE_HEIGHT"].data))
                    camera_bp.set_attribute("fov", str(self.properties["CAMERA_IMAGE_FOV"].data))

                    self.cameraFront = self.world.world.spawn_actor(blueprint=camera_bp, transform=carla.Transform(
                        carla.Location(x=self.properties["CAMERA_LOCATION_X"].data,
                                       y=self.properties["CAMERA_LOCATION_Y"].data,
                                       z=self.properties["CAMERA_LOCATION_Z"].data),
                        carla.Rotation(roll=self.properties["CAMERA_LOCATION_RX"].data,
                                       pitch=self.properties["CAMERA_LOCATION_RY"].data,
                                       yaw=self.properties["CAMERA_LOCATION_RZ"].data)), attach_to=self.world.player)

                    self.cameraFront.listen(lambda data: sensor_callback(data, self.image_queue))
                else:
                    self.cameraFront = None

                # LIDAR world.get_blueprint_library()
                if self.properties["lidarEnabled"].data:

                    blueprint_LID = self.world.world.get_blueprint_library().find('sensor.lidar.ray_cast')
                    blueprint_LID.set_attribute('range', str(int(
                        self.properties["LIDAR_MAX_RANGE_IN_M"].data * 1000.0)))  # '50000')
                    blueprint_LID.set_attribute('channels', str(int(self.properties["LIDAR_NB_LAYERS"].data)))  # '16')

                    blueprint_LID.set_attribute('rotation_frequency',
                                                str(int(self.properties["LIDAR_ROTATION_FREQUENCY_HZ"].data)))  # '20')

                    blueprint_LID.set_attribute('upper_fov',
                                                str(self.properties["LIDAR_UPPER_FOV_IN_DEG"].data))  # '15')
                    blueprint_LID.set_attribute('lower_fov',
                                                str(self.properties["LIDAR_LOWER_FOV_IN_DEG"].data))  # '-15')
                    blueprint_LID.set_attribute('noise_stddev',
                                                str(self.properties["LIDAR_NOISE_IN_M"].data))  # '0.02')#add noise

                    self.lidar = self.world.world.spawn_actor(
                        blueprint=blueprint_LID,
                        transform=carla.Transform(carla.Location(x=self.properties["LIDAR_CAR_LOCATION_X"].data,
                                                                 y=self.properties["LIDAR_CAR_LOCATION_Y"].data,
                                                                 z=self.properties["LIDAR_CAR_LOCATION_Z"].data),
                                                  carla.Rotation(pitch=self.properties["LIDAR_CAR_LOCATION_RY"].data,
                                                                 roll=self.properties["LIDAR_CAR_LOCATION_RX"].data,
                                                                 yaw=self.properties["LIDAR_CAR_LOCATION_RZ"].data)),

                        attach_to=self.world.player)

                    self.lidar_queue = Queue()
                    self.lidar.listen(lambda data: sensor_callback(data, self.lidar_queue))
                else:
                    self.lidar = None

        try:
            self.inputs["in"]
        except:
            print("error")
            return  # no data in fifo
        dataRtmaps = self.inputs["in"].ioelt.data

        self.clock.tick_busy_loop(60)
        # update carla control
        if len(dataRtmaps) >= 5:
            self.controller.parseRtmaps(self.world, dataRtmaps)

        self.world.tick(self.clock)
        self.world.render(self.display)
        pygame.display.flip()

        if self.properties["cameraEnabled"].data:
            while self.image_queue.qsize() > 0:  # clear the queue
                image_data = ""
                try:
                    image_data = self.image_queue.get(False)
                except Empty:
                    print("[Warning] Some sensor data has been missed")

                if len(image_data) > 0:
                    self.outputs["imageCamera"].write(image_data.raw_data)

        if self.properties["lidarEnabled"].data:
            while self.lidar_queue.qsize() > 0:  # clear the queue
                lidar_data = ""
                try:
                    lidar_data = self.lidar_queue.get(False)
                except Empty:
                    print("[Warning] Some sensor data has been missed")
                if len(lidar_data) > 0:

                    timeStampUs = rt.current_time()
                    if self.properties["UseSimulationTimeStamp"].data:
                        timeStampUs = int(lidar_data.timestamp * 1000000.0)

                    p_cloud = np.copy(np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4')))
                    points = np.reshape(p_cloud, (int(p_cloud.shape[0] / 4), 4))  # x y z intensity
                    # mise dans les axes main droite des donnes lidar
                    p_cloud = np.zeros((points.shape[0], 3))
                    p_cloud[:, 0] = points[:, 0]  # y
                    p_cloud[:, 1] = -points[:, 1]  # x
                    p_cloud[:, 2] = points[:, 2]  # -z

                    p_cloud = p_cloud.reshape(-1).astype("float64")
                    self.outputs["ptCloud"].write(p_cloud, ts=timeStampUs)

        speedKmh, accx3, gyrox3, imu3, roll, pitch, yaw, gnssTS = \
            getPose(self.world)
        timeStampUs = rt.current_time()

        if self.properties["UseSimulationTimeStamp"].data:
            timeStampUs = int(gnssTS * 1000000.0)

        # send output
        # recordPose(self.world, self.wb, self.ws)
        gps_x = self.world.player.get_transform().location.x
        gps_y = self.world.player.get_transform().location.y
        gps_yaw = self.world.player.get_transform().rotation.yaw
        self.outputs["state"].write(np.array([gps_x, gps_y, gps_yaw]), ts=timeStampUs)

        self.outputs["speedKmh"].write(np.array([speedKmh]), ts=timeStampUs)
        self.outputs["accXYZ"].write(np.array(accx3), ts=timeStampUs)
        self.outputs["gyroXYZ"].write(np.array(gyrox3), ts=timeStampUs)
        self.outputs["imuLatLongAlt"].write(np.array(imu3), ts=timeStampUs)
        self.outputs["rollPitchYaw"].write(np.array([roll, pitch, (90+yaw)]), ts=timeStampUs)

    def spawnVehicles(self, number_of_vehicles):
        self.traffic_manager = self.client.get_trafficmanager(int(self.properties["TM_PORT"].data))
        self.traffic_manager.set_global_distance_to_leading_vehicle(1.0)
        self.traffic_manager.set_hybrid_physics_mode(True)

        synchronous_master = False

        self.vehicles_list = []

        blueprints = self.world.world.get_blueprint_library().filter('vehicle.*')
        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        SetVehicleLightState = carla.command.SetVehicleLightState

        spawn_points = self.world.world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        if number_of_vehicles < number_of_spawn_points:
            random.shuffle(spawn_points)
        elif number_of_vehicles > number_of_spawn_points:
            print("warning  requested " + str(number_of_vehicles) + " vehicles, number of spawn points is " + str(
                number_of_spawn_points))
            number_of_vehicles = number_of_spawn_points
        batch = []
        for n, transform in enumerate(spawn_points):
            if n >= number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
            blueprint.set_attribute('role_name', 'autopilot')

            # prepare the light state of the cars to spawn
            # light_state = vls.NONE
            if True:  # args.car_lights_on:
                light_state = vls.Position | vls.LowBeam | vls.LowBeam

            # spawn the cars and set their autopilot and light state all together
            batch.append(SpawnActor(blueprint, transform)
                         .then(SetAutopilot(FutureActor, True, self.traffic_manager.get_port()))
                         .then(SetVehicleLightState(FutureActor, light_state)))

        for response in self.client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                print(response.error)  # logging.error(response.error)
            else:
                self.vehicles_list.append(response.actor_id)

        # example of how to use parameters
        self.traffic_manager.global_percentage_speed_difference(30.0)

    def spawnWalker(self, number_of_walkers):
        percentagePedestriansRunning = 0.5  # how many pedestrians will run
        percentagePedestriansCrossing = 0.0  # how many pedestrians will walk through the road

        SpawnActor = carla.command.SpawnActor
        # 1. take all the random locations to spawn
        blueprintsWalkers = self.world.world.get_blueprint_library().filter('walker.pedestrian.*')
        self.walkers_list = []
        self.all_id = []
        # 1. take all the random locations to spawn
        spawn_points = []
        for i in range(number_of_walkers):
            spawn_point = carla.Transform()
            loc = self.world.world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point.location = loc
                spawn_points.append(spawn_point)

        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if random.random() > percentagePedestriansRunning:
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, spawn_point))
        results = self.client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                print(results[i].error)  # logging.error(results[i].error)
            else:
                self.walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2

        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = self.world.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(self.walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), self.walkers_list[i]["id"]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                print(results[i].error)  # logging.error(results[i].error)
            else:
                self.walkers_list[i]["con"] = results[i].actor_id

        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(self.walkers_list)):
            self.all_id.append(self.walkers_list[i]["con"])
            self.all_id.append(self.walkers_list[i]["id"])
        self.all_actors = self.world.world.get_actors(self.all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created

        self.world.world.tick()
        # 5. initialize each controller and set target to walk to (list is [controller, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        self.world.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(self.all_id), 2):
            # start walker
            self.all_actors[i].start()
            # set walk to random point
            self.all_actors[i].go_to_location(self.world.world.get_random_location_from_navigation())
            # max speed
            self.all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))

    def Death(self):
        # stop walker and vehicles
        print('\nDestroying %d vehicles' % len(self.vehicles_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.vehicles_list])

        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(self.all_id), 2):
            self.all_actors[i].stop()

        print('\nDestroying %d walkers' % len(self.walkers_list))
        self.client.apply_batch([carla.command.DestroyActor(x) for x in self.all_id])

        # destroy world
        if self.world and self.world.recording_enabled:
            self.client.stop_recorder()

        if self.world is not None:
            self.world.destroy()

        if self.lidar is not None:
            self.lidar.destroy()

        if self.cameraFront is not None:
            self.cameraFront.destroy()

        pygame.quit()

        pass
