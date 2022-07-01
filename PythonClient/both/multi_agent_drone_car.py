import setup_path
import airsim
import cv2

import time
import numpy as np
import os
import tempfile
import pprint

# Use below in settings.json with Blocks environment
"""
{
    "SettingsVersion": 1.2,
    "SimMode": "Both",

    "Vehicles": {
        "Car1": {
          "VehicleType": "PhysXCar",
          "X": 0, "Y": 0, "Z": -2
        },
        "Drone1": {
          "VehicleType": "SimpleFlight",
          "X": 0, "Y": 0, "Z": -5,
      "Yaw": 90
        }
  }
}

"""

# connect to the AirSim simulator
client_1 = airsim.MultirotorClient(port=41451)
client_1.confirmConnection()
client_1.enableApiControl(True, "Drone1")
client_1.armDisarm(True, "Drone1")

client_2 = airsim.CarClient(port=41452)
client_2.confirmConnection()
client_2.enableApiControl(True, "Car1")
car_controls1 = airsim.CarControls()

airsim.wait_key('Both clients connected, press any key to begin movement')

f1 = client_1.takeoffAsync(vehicle_name="Drone1")

car_state1 = client_2.getCarState("Car1")
print("Car1: Speed %d, Gear %d" % (car_state1.speed, car_state1.gear))
f1.join()
state1 = client_1.getMultirotorState(vehicle_name="Drone1")
s = pprint.pformat(state1)
print("Drone1: State: %s" % s)

f1 = client_1.moveToPositionAsync(-5, 5, -10, 5, vehicle_name="Drone1")
car_controls1.throttle = 0.5
car_controls1.steering = 0.5
client_2.setCarControls(car_controls1, "Car1")
print("Car1: Go Forward")
f1.join()

airsim.wait_key('Press any key to reset to original state')

client_1.armDisarm(False, "Drone1")
client_1.reset()
client_2.reset()

# that's enough fun for now. let's quit cleanly
client_1.enableApiControl(False, "Drone1")
client_2.enableApiControl(False, "Car1")
