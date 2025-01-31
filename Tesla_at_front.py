#!/usr/bin/env python

import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse
import logging
from numpy import random


def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []


def spawn_vehicle_in_front_of_ego_vehicle(world, blueprint_library, ego_vehicle, distance=75, retries=5):
    """
    Spawns a Tesla Model 3 vehicle at a specified distance in front of the ego vehicle and keeps it stationary.
    Retries with slight vertical offset if collision occurs.
    """
    ego_transform = ego_vehicle.get_transform()  # Get the ego vehicle's transform
    forward_vector = ego_transform.get_forward_vector()  # Direction the ego vehicle is facing

    # Get the Tesla Model 3 blueprint
    vehicle_blueprint = blueprint_library.find('vehicle.tesla.model3')

    if vehicle_blueprint.has_attribute('color'):
        color = random.choice(vehicle_blueprint.get_attribute('color').recommended_values)
        vehicle_blueprint.set_attribute('color', color)

    vehicle_blueprint.set_attribute('role_name', 'stationary')

    for i in range(retries):
        spawn_location = ego_transform.location + forward_vector * distance
        spawn_location.z += i * 0.5  # Increase height slightly each retry to avoid collision

        spawn_transform = carla.Transform(spawn_location, ego_transform.rotation)

        # Try to spawn the vehicle
        vehicle = world.try_spawn_actor(vehicle_blueprint, spawn_transform)
        if vehicle is not None:
            print(f"Spawned vehicle {vehicle.id} in front of the ego vehicle (stationary) after {i + 1} attempt(s).")
            return vehicle

    print("Failed to spawn the vehicle after multiple attempts.")
    return None




def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='Avoid spawning vehicles prone to accidents')
    argparser.add_argument(
        '--filterv',
        metavar='PATTERN',
        default='vehicle.*',
        help='Filter vehicle model (default: "vehicle.*")')
    argparser.add_argument(
        '--generationv',
        metavar='G',
        default='All',
        help='restrict to certain vehicle generation (values: "1","2","All" - default: "All")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')

    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    vehicles_list = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False

    try:
        world = client.get_world()

        settings = world.get_settings()
        if not args.asynch:
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05
            else:
                synchronous_master = False
        else:
            print("You are currently in asynchronous mode. If this is a traffic simulation, \
            you could experience some issues. If it's not working correctly, switch to synchronous \
            mode by using traffic_manager.set_synchronous_mode(True)")

        world.apply_settings(settings)

        blueprints = get_actor_blueprints(world, args.filterv, args.generationv)

        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
            blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('t2')]
            blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
            blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
            blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

        # -------------------
        # Identify the ego vehicle
        # -------------------
        ego_vehicle = None
        for vehicle in world.get_actors().filter('vehicle.*'):
            if 'ego_vehicle' in vehicle.attributes.get('role_name', ''):
                ego_vehicle = vehicle
                break

        if ego_vehicle is None:
            print("Ego vehicle not found!")
            return

        # Spawn a vehicle 75 meters in front of the ego vehicle and keep it stationary
        new_vehicle = spawn_vehicle_in_front_of_ego_vehicle(world, world.get_blueprint_library(), ego_vehicle)
        vehicles_list.append(new_vehicle.id)
        print(f"Spawned vehicle {new_vehicle.id} in front of the ego vehicle (stationary).")

        while True:
            if not args.asynch and synchronous_master:
                world.tick()
            else:
                world.wait_for_tick()

    finally:
        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

        time.sleep(0.5)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
