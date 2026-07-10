import math

from controller import Supervisor, Keyboard
import numpy as np
import cv2

from localization.april_tags import AprilTagDetector, TagLocalizer, build_tag_world, tag_world_from_sim
from localization.imu import IMU

from navigation.keyboard import move_from_key_input
from navigation.ball_navigation import BallNavigator, nearest_point

# real-time spawning logic (stays in)
from webots_env.ball_spawner import spawn_balls
from webots_env.april_tags.april_tag_spawner import spawn_tags

# goal is to have no webots_env imports in the final simulation
# localization, aboslute position of the robot should be replaced with odometry logic
from webots_env.verify import find_nearest_ball_to_coord, robot_pose_from_supervisor
from webots_env.visualize import plot_nodes
from webots_env.camera_offset import measure_camera_offset, offset_from_residual

TIME_STEP = 32 # test the limits of the time step with GPU

robot = Supervisor()

# keyboard initialization
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# Motor initialization
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

MAX_SPEED = 6.28

# Camera Initialization
camera = robot.getDevice('front_camera')
camera.enable(TIME_STEP)

width, height = camera.getWidth(), camera.getHeight()

camera_fov = camera.getFov()
camera_pitch_down = 0.1309 # lower pitch down (horizontal) makes metal ball detection worse
camera_height = 0.16 # (meters)

# off_fwd, off_right = measure_camera_offset(robot, camera_def="CAM", forward_axis=0)
# print("camera offset forward, right =", off_fwd, off_right)

# spawn balls
METAL_COUNT = 24
ORANGE_COUNT = 16
spawn_balls(robot, metal_count=METAL_COUNT, orange_count=ORANGE_COUNT, arena_size=2.0) # maybe softcode the arena size

# spawn april tags
spawn_tags(robot)
robot.step(TIME_STEP)

# april tag detector
april_tag_detector = AprilTagDetector(fov=camera_fov)

# tag_locs = build_tag_world()
tag_locs = tag_world_from_sim(robot)
lozalizer = TagLocalizer(tag_locs, cam_pitch_down=camera_pitch_down, cam_height=camera_height, cam_offset_forward=0.0)

imu = IMU(robot, name='inertial_unit')
robot.step(TIME_STEP)

# navigation
navigator = BallNavigator(max_wheel_speed=MAX_SPEED)



# simulation loop
while robot.step(TIME_STEP) != -1:

    # raw  = imu.roll_pitch_yaw()[2]
    # true = robot_pose_from_supervisor(robot)[2]
    # d = math.atan2(math.sin(true - raw), math.cos(true - raw))
    # print(f"imu raw={math.degrees(raw):+.1f}  true={math.degrees(true):+.1f}  diff={math.degrees(d):+.1f} deg")

    # Read camera after a single getkey
    image = camera.getImage()

    img = np.frombuffer(image, dtype=np.uint8)
    img = img.reshape((height, width, 4))

    img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    frame = img_bgr # keep convention


    # Localization

    dets = april_tag_detector.detect(frame)

    yaw = imu.yaw()
    robot_xs, robot_ys = lozalizer.robot_position(dets, yaw, debug=False) # returns None if we can't see any AprilTags

    robot_positions = list(zip(robot_xs, robot_ys))
    robot_positions.append((np.mean(robot_xs).item(), np.mean(robot_ys).item()))

    plot_nodes(robot, robot_positions, predicted_position=None, points_are_world=True, orange_balls_count=len(robot_xs))

    # Navigation

    left, right = move_from_key_input(keyboard, MAX_SPEED)
    left_motor.setVelocity(left)
    right_motor.setVelocity(right)
