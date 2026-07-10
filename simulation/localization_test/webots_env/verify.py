import math

FORWARD_AXIS = 0

def robot_pose_from_node(node):
    """Return (robot_x, robot_y, yaw) for a robot node in an ENU (z-up) world.

    robot_x, robot_y : ground position from the translation field.
    yaw              : heading of the robot's forward axis in the world ground
                       plane, measured from +X toward +Y, in radians.
    """
    robot_x, robot_y, _ = node.getField("translation").getSFVec3f()

    o = node.getOrientation()
    fwd_world_x = o[FORWARD_AXIS]
    fwd_world_y = o[3 + FORWARD_AXIS]
    yaw = math.atan2(fwd_world_y, fwd_world_x)
    return robot_x, robot_y, yaw

def robot_pose_from_supervisor(supervisor, children_field=None, num_nodes=None):
    # takes in the supervisor node and finds the 

    if children_field is None or num_nodes is None:
        root = supervisor.getRoot()
        children_field = root.getField('children')
        num_nodes = children_field.getCount()
    
    robot_node = None
    for i in range(num_nodes):
        node = children_field.getMFNode(i)
        type_name = node.getTypeName()
        if type_name and "TurtleBot" in type_name:
            robot_node = node
            break

    if robot_node is None:
        return  # robot not found, skip frame

    return robot_pose_from_node(robot_node)

def body_to_world(origin_x, origin_y, yaw, right, forward):
    """Map a body-frame point (right, forward) to world (x, y).

    Implements world = origin + Rz(yaw) . body. With a z-up world the robot's
    forward axis is ( cos yaw,  sin yaw) and its right axis is ( sin yaw,
    -cos yaw), so:
        world_x = origin_x + forward*cos(yaw) + right*sin(yaw)
        world_y = origin_y + forward*sin(yaw) - right*cos(yaw)
    """
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    world_x = origin_x + forward * cos_y + right * sin_y
    world_y = origin_y + forward * sin_y - right * cos_y
    return world_x, world_y

def find_nearest_ball_to_coord(supervisor, predicted_x, predicted_z):
    root = supervisor.getRoot()
    children_field = root.getField('children')
    num_nodes = children_field.getCount()

    robot_pos = None
    for i in range(num_nodes):
        node = children_field.getMFNode(i)
        def_name = node.getTypeName()
        if def_name and "TurtleBot" in def_name:
            trans_field = node.getField("translation")
            if trans_field:
                robot_pos = trans_field.getSFVec3f()
                break
                
    if robot_pos is None:
        print("⚠️ Robot not found in scene tree.")
        return (0.0, 0.0), float('inf')
        
    robot_x, _, robot_z = robot_pos

    min_error = float('inf')
    best_ball_dist = (0.0, 0.0)

    for i in range(num_nodes):
        node = children_field.getMFNode(i)
        
        name_field = node.getField("name")
        node_name = name_field.getSFString() if name_field else ""
        
        if node_name.startswith(("metalball_", "orangeball_")):
            trans_field = node.getField("translation")
            if not trans_field:
                continue
                
            ball_pos = trans_field.getSFVec3f()
            ball_x, _, ball_z = ball_pos
            
            rel_x = ball_x - robot_x
            rel_z = ball_z - robot_z
            
            error = math.hypot(rel_x - predicted_x, rel_z - predicted_z)
            
            if error < min_error:
                min_error = error
                best_ball_dist = (rel_x, rel_z)
                
    return best_ball_dist, min_error