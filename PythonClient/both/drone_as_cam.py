import setup_path
import airsim
import cv2

import time
import numpy as np
import os
import tempfile
import pprint
import math

def getColorImages(client, fp, name):
  rawImage = client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)], name)[0]
  if (rawImage == None):
    print("Camera is not working")
  else:
    img1d = np.frombuffer(rawImage.image_data_uint8, dtype=np.uint8)
    png = img1d.reshape(rawImage.height, rawImage.width, 3)
    cv2.imshow("DroneCam", png)
    cv2.setWindowProperty("DroneCam", cv2.WND_PROP_TOPMOST, 1)
  return cv2.waitKeyEx(1)

'''
@return (throttle, steering, is_manual_gear, manual_gear, key)
'''
def getImages(client, fp, name):
  # result = client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthVis, False, False)], name)[0]
  result, = client.simGetImages(
    [
      airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False)
    ], vehicle_name="Drone1"
  )
  drive = [0, 0, False, 1]
  if (result == None):
    print("Camera is not working")
  else:
    depth_image = airsim.list_to_2d_float_array(result.image_data_float, result.width, result.height)
    depth_image = depth_image.reshape(result.height, result.width, 1)

    depth_lerp = np.interp(depth_image, (0, 100), (0, 255))

    # top[top == 255] =  np.nan
    # top = np.vsplit(gray, 2)[1] # Used to look only at things below the drone and not straight ahead
    bands = np.hsplit(depth_image, [50, 100, 150, 200])
    medians = [np.median(x) for x in bands]
    max = np.argmax(medians)
    # sums = bands.copy()
    # sums = [(x > 5).sum() for x in sums]
    # best_candidate = np.argmax(sums)

    distance = medians[max]
    current = medians[2]

    if (current < 5):
      drive = [.2, .4, True, -1]
    else:
      if (max == 0):
        drive = [.2, -.2, False, 1]
      elif (max == 1):
        drive = [.2, -.1, False, 1]
      elif (max == 2):
        drive = [.2, 0, False, 1]
      elif (max == 3):
        drive = [.2, .1, False, 1]
      else:
        drive = [.2, .2, False, 1]

    x = int(max * 50)
    cv2.rectangle(depth_lerp, (x, 0), (x+50, depth_lerp.shape[0]), (0, 255, 0), 2)
    cv2.imshow("DroneCam", depth_lerp.astype('uint8'))
    cv2.setWindowProperty("DroneCam", cv2.WND_PROP_TOPMOST, 1)
    key = cv2.waitKeyEx(1)
    drive.append(key)
    return drive
  return [0, 0, False, 1, -1]

def teleOp(key, car_controls1):
    if (key == 2490368):
      car_controls1.throttle += .1
      car_controls1.is_manual_gear = False
    elif (key == 2621440):
      car_controls1.throttle -= .1
      car_controls1.is_manual_gear = True
      car_controls1.manual_gear = -1
    elif (key == 2555904):
      car_controls1.steering += .1
      car_controls1.is_manual_gear = False
    elif (key == 2424832):
      car_controls1.steering -= .1
      car_controls1.is_manual_gear = False
    elif (key == 2228224):
      car_controls1.throttle -= .1
      car_controls1.is_manual_gear = False
    elif (key == 100):
      print("Drone is attempting to go to: {}\n Drone is currently at: {}\n Prev Moving: {}\n Tasks {}\n Car State: {}".format(car_pos, drone_pos, prev_moving, tasks, car.getCarState()))
    elif (key != -1):
      car_controls1.throttle = 0
      car_controls1.steering = 0
      car_controls1.is_manual_gear = False
      print("Key Num: {}".format(key))

    return car_controls1


if __name__ == "__main__":
  # connect to the AirSim simulator
  angle = 0.785398
  delta = 0.3
  camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(angle, 0, 0))  #PRY in radians
  drone = airsim.MultirotorClient(port=41451)
  drone.confirmConnection()
  drone.enableApiControl(True, "Drone1")
  drone.armDisarm(True, "Drone1")
  drone.simSetCameraPose("0", camera_pose)

  car = airsim.CarClient(port=41452)
  car.confirmConnection()
  car.enableApiControl(True, "Car1")
  car_controls1 = airsim.CarControls()

  airsim.wait_key('Both clients connected, press any key to begin movement')

  f1 = drone.takeoffAsync(vehicle_name="Drone1").join()
  key = getImages(drone, "", "Drone1")
  path = []
  start_car_state = car.simGetVehiclePose("Car1")
  start_car_pos = start_car_state.position
  start_drone_state = drone.simGetVehiclePose("Drone1")
  start_drone_pos = start_drone_state.position
  prev_moving = False
  tasks = []

  while(key != 113):
    drive = getImages(drone, "", "Drone1")
    car_state = car.simGetVehiclePose("Car1")
    car_pos = car_state.position - start_car_pos
    car_ang = airsim.to_eularian_angles(car_state.orientation)

    drone_state = drone.simGetVehiclePose("Drone1")
    drone_pos = drone_state.position - start_drone_pos

    diff_pos = (car_pos)
    delta_pos = (car_pos - drone_pos)
    if (car.getCarState().speed > 0):
      delta = .1
    else:
      delta = 1
    if (abs(delta_pos.x_val) > delta or  abs(delta_pos.y_val) > delta):
      drone.moveToPositionAsync(float(diff_pos.x_val), float(diff_pos.y_val), float(diff_pos.z_val)-1, 2,
                                      drivetrain = airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode = airsim.YawMode(False, math.degrees(car_ang[2])), vehicle_name="Drone1",
                                      lookahead = -1, adaptive_lookahead = 1).join()
      if(not prev_moving):
        print("Moving!")
        prev_moving = True

    car_controls1.throttle = drive[0]
    car_controls1.steering = drive[1]
    car_controls1.is_manual_gear = drive[2]
    car_controls1.manual_gear = drive[3]
    key = drive[4]
    car.setCarControls(car_controls1, "Car1")

  airsim.wait_key('Press any key to reset to original state')

  drone.armDisarm(False, "Drone1")
  drone.reset()
  car.reset()

  # that's enough fun for now. let's quit cleanly
  drone.enableApiControl(False, "Drone1")
  car.enableApiControl(False, "Car1")
