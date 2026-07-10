from controller import Supervisor, Keyboard
import numpy as np
import cv2

from perception.detector import CombinedBallDetector
from perception.ground_projector import GroundProjector
from perception.ball_tracker import BallTracker

from localization.april_tags import AprilTagDetector, TagLocalizer, build_tag_world
from localization.imu import IMU

from navigation.keyboard import move_from_key_input
from navigation.ball_navigation import BallNavigator, nearest_point
from navigation.launch_fsm import LaunchFSM

from collection.collection_mechanism import BallCollector, _within

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

# ball detector
detector = CombinedBallDetector()
projector = GroundProjector(
    fov_h_rad=camera.getFov(),
    width=width,
    height=height,
    cam_height=camera_height,
    pitch_down_rad=camera_pitch_down,
)
orange_tracker = BallTracker(projector)
metal_tracker = BallTracker(projector)

# april tag detector
april_tag_detector = AprilTagDetector(fov=camera_fov)

tag_locs = build_tag_world()

# IMPORTANT
# recalibrate cam_offset_forward (it's close to 0.0 but not exactly)
# run Claude's function again and get a very small positive value (less than 0.05)
lozalizer = TagLocalizer(tag_locs, cam_pitch_down=camera_pitch_down, cam_height=camera_height, cam_offset_forward=0.03)

imu = IMU(robot, name='inertial_unit')
robot.step(TIME_STEP)

# navigation
navigator = BallNavigator(max_wheel_speed=MAX_SPEED)
fsm = None

# collection
collector = BallCollector(robot)
collected_balls = [] # keep tracks of the balls collected


# navigation FSM
_state_seeking, _state_advancing = 0, 1
_state = _state_seeking

advance_start_time = None
advance_target = None

advance_straight = 1.5 # seconds


# simulation loop
while robot.step(TIME_STEP) != -1:

    # Read camera after a single getkey
    image = camera.getImage()

    img = np.frombuffer(image, dtype=np.uint8)
    img = img.reshape((height, width, 4))

    img_bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    frame = img_bgr # keep convention


    # Localization

    dets = april_tag_detector.detect(frame)

    yaw = imu.yaw()
    robot_pos_predicted = lozalizer.robot_position(dets, yaw, debug=False) # returns None if we can't see any AprilTags
    
    if robot_pos_predicted is None: # too close to walls
        print("No April Tags detected")
        left_motor.setVelocity(-MAX_SPEED)
        right_motor.setVelocity(MAX_SPEED)
    
        continue

    robot_x, robot_y = robot_pos_predicted # override webots_env-generated variables


    # Perception (ball detection)

    detections = detector.detect(frame)

    orange_balls = orange_tracker.update(detections['orange'], (robot_x, robot_y, yaw))
    metal_balls = metal_tracker.update(detections['metal'], (robot_x, robot_y, yaw))

    ball_pos = [
        pos for _, pos in orange_balls + metal_balls if pos is not None and (not any(_within(pos, c, 0.08) for c in collected_balls))
    ]

    # plot_nodes(robot, ball_pos, predicted_position=robot_pos_predicted, points_are_world=True, orange_balls_count=len(orange_balls))

    # if not ball_pos: # no balls detected
    #     print("No balls detected")
    #     left_motor.setVelocity(-MAX_SPEED)
    #     right_motor.setVelocity(MAX_SPEED)
    
    #     continue


    # Navigation

    # if fsm is None:
    #     fsm = LaunchFSM(box_position=(robot_x, robot_y - 0.25))

    # pose = (robot_x, robot_y, yaw)

    print(robot_x, robot_y, yaw)

    # nearest_ball = nearest_point((robot_x, robot_y), ball_pos)
    # collected = collector.check_and_collect()

    # if collected is not None and nearest_ball is not None:
    #     collected_balls.append(nearest_ball)

    # left, right = fsm.step(pose, ball_pos, collected, robot.getTime())

    left, right = move_from_key_input(keyboard, MAX_SPEED)

    left_motor.setVelocity(left)
    right_motor.setVelocity(right)



    # Old navigation

    # # if arrived, then remove the closet ball to the current position
    # collected_ball_pos = collector.check_and_collect()

    # if _state == _state_seeking:
    #     nearest_ball = nearest_point((robot_x, robot_y), ball_pos)
    #     (left, right), arrived = navigator.step((robot_x, robot_y, yaw), nearest_ball)

    #     print(left, right, arrived)

    #     if arrived:
    #         _state = _state_advancing
    #         advance_start_time = robot.getTime()
    #         advance_target = nearest_ball

    # else:
    #     left, right = MAX_SPEED, MAX_SPEED
    #     timed_out = robot.getTime() - advance_start_time > advance_straight

    #     print(collected_ball_pos, timed_out)

    #     if collected_ball_pos is not None or timed_out:
    #         if advance_target is not None:
    #             collected_balls.append(advance_target)

    #         _state = _state_seeking
    #         left, right = 0.0, 0.0

    # # print(_state, left, right)

    # left_motor.setVelocity(left)
    # right_motor.setVelocity(right)
