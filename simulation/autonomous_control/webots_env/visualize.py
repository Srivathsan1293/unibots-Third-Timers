import math
import matplotlib.pyplot as plt

from webots_env.verify import robot_pose_from_supervisor, body_to_world

CAM_OFFSET_FORWARD = 0.0
CAM_OFFSET_RIGHT   = 0.0

# Global figure state (persists across calls)
_fig = None
_ax = None
_scatter_pred = None
_scatter_robot = None
_scatter_robot_pred = None
_scatter_balls = None

_scatter_orange_pred = None
_scatter_orange_balls = None
_scatter_metal_pred = None
_scatter_metal_balls = None

def plot_nodes(supervisor, predicted_points, predicted_position=None, points_are_world=False, orange_balls_count=None):
    # points_are_world means predicted_points are already absolute / global

    global _fig, _ax, _scatter_robot, _scatter_robot_pred, _scatter_pred, _scatter_balls
    global _scatter_orange_pred, _scatter_orange_balls, _scatter_metal_pred, _scatter_metal_balls
    
    if _fig is None:
        plt.ion()  # interactive mode
        _fig, _ax = plt.subplots(figsize=(10, 10))
        _ax.set_title("Ball Detection - Real-Time")
        _ax.set_xlabel("X (meters)")
        _ax.set_ylabel("Z (meters)")
        _ax.grid(True)
        _ax.set_aspect('equal')
        
        _scatter_robot, = _ax.plot([], [], 'gs', label='Robot', markersize=12)

        if predicted_position is not None:
            _scatter_robot_pred, = _ax.plot([], [], 'ys', label='Robot predicted', markersize=12)

        if orange_balls_count is None:
            _scatter_pred, = _ax.plot([], [], 'ro', label='Predicted', markersize=8)
            _scatter_balls, = _ax.plot([], [], 'go', label='Balls', markersize=6)
        
        else:
            _scatter_orange_pred, = _ax.plot([], [], 'mo', label='Orange Predicted', markersize=8)
            _scatter_orange_balls, = _ax.plot([], [], 'ro', label='Orange Balls', markersize=6)

            _scatter_metal_pred, = _ax.plot([], [], 'co', label='Metal Predicted', markersize=8)
            _scatter_metal_balls, = _ax.plot([], [], 'bo', label='Metal Balls', markersize=6)

        _ax.legend()
    
    
    # Get robot position
    # necessary for absolute positions of balls (ground truth labels)
    root = supervisor.getRoot()
    children_field = root.getField('children')
    num_nodes = children_field.getCount()

    robot_x, robot_y, yaw = robot_pose_from_supervisor(supervisor, children_field=children_field, num_nodes=num_nodes)

    cam_x, cam_y = body_to_world(robot_x, robot_y, yaw, CAM_OFFSET_RIGHT, CAM_OFFSET_FORWARD)

    if points_are_world:
        pred_x = [p[0] for p in predicted_points]
        pred_y = [p[1] for p in predicted_points]

    else:
        # predicted points are relative to the robot (accounts for yaw)
        pred_x, pred_y = [], []
        for p in predicted_points:        # p[0] = right, p[1] = forward
            wx, wy = body_to_world(cam_x, cam_y, yaw, p[0], p[1])
            pred_x.append(wx)
            pred_y.append(wy)

    # get ball positions
    orange_ball_x, orange_ball_y, metal_ball_x, metal_ball_y = [], [], [], []

    for i in range(num_nodes):
        node = children_field.getMFNode(i)
        name_field = node.getField("name")
        node_name = name_field.getSFString() if name_field else ""
        
        if node_name.startswith("orangeball_"):
            trans_field = node.getField("translation")

            if trans_field:
                bx, by, _ = trans_field.getSFVec3f()
                orange_ball_x.append(bx)
                orange_ball_y.append(by)

        elif node_name.startswith("metalball_"):
            trans_field = node.getField("translation")

            if trans_field:
                bx, by, _ = trans_field.getSFVec3f()
                metal_ball_x.append(bx)
                metal_ball_y.append(by)

    ball_x = orange_ball_x + metal_ball_x
    ball_y = orange_ball_y + metal_ball_y
    
    # Update scatter plots
    _scatter_robot.set_data([robot_x], [robot_y])

    if predicted_position is not None:
        _scatter_robot_pred.set_data([predicted_position[0]], [predicted_position[1]])

    if orange_balls_count is None:
        _scatter_pred.set_data(pred_x, pred_y)
        _scatter_balls.set_data(ball_x, ball_y)

    else:
        _scatter_orange_pred.set_data(pred_x[:orange_balls_count], pred_y[:orange_balls_count])
        _scatter_orange_balls.set_data(orange_ball_x, orange_ball_y)

        _scatter_metal_pred.set_data(pred_x[orange_balls_count:], pred_y[orange_balls_count:])
        _scatter_metal_balls.set_data(metal_ball_x, metal_ball_y)
    
    # dynamic axis limits
    # all_x = pred_x + [robot_x] + ball_x
    # all_y = pred_y + [robot_y] + ball_y
    # if all_x and all_y:
    #     margin = 0.5
    #     _ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    #     _ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    # hardcode arena side as 2x2 square centered at 0
    _ax.set_xlim(-1.0, 1.0)
    _ax.set_ylim(-1.0, 1.0)

    # Update display without blocking
    _fig.canvas.draw()
    _fig.canvas.flush_events()
    plt.pause(0.001)  # Critical: allows GUI to update
