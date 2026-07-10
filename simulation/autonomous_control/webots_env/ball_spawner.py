import random
import math

import random
import math

def spawn_balls(supervisor, metal_count, orange_count, arena_size,
                ball_radius=0.04, padding=0.01, robot_def="ROBOT", 
                robot_exclusion_radius=0.4, max_attempts=10000):
    
    root = supervisor.getRoot()
    children = root.getField("children")

    min_dist = 2 * ball_radius + padding

    placed = []

    robot_node = supervisor.getFromDef(robot_def)
    if robot_node is None:
        raise ValueError(f"Robot with DEF '{robot_def}' not found in the world.")
    
    robot_field = robot_node.getField("translation")
    robot_x, robot_z, _ = robot_field.getSFVec3f() # really bad convention
    robot_pos_2d = (robot_x, robot_z)

    def valid(x, z):
        d_robot = math.hypot(x - robot_pos_2d[0], z - robot_pos_2d[1])
        if d_robot < robot_exclusion_radius:
            return False

        for px, pz in placed:
            d_ball = math.hypot(x - px, z - pz)
            if d_ball < min_dist:
                return False
                
        return True

    def random_pos():
        for _ in range(max_attempts):
            x = random.uniform(-arena_size / 2, arena_size / 2)
            z = random.uniform(-arena_size / 2, arena_size / 2)
            
            if valid(x, z):
                placed.append((x, z))
                return x, z
                
        raise RuntimeError(
            f"Failed to find valid spawn position after {max_attempts} attempts. "
            "Try reducing ball counts, lowering exclusion radius, or increasing arena size."
        )

    def spawn(proto, x, z, name):
        y = 0.02 if proto == "MetalBall" else 0.03
        
        node = f'{proto} {{ translation {x:.4f} {z:.4f} {y:.4f} name "{name}" }}'.strip() # pay attention to format (x, z, y)
        children.importMFNodeFromString(-1, node)

    for i in range(metal_count):
        x, z = random_pos()
        spawn("MetalBall", x, z, f"metalball_{i}")

    for i in range(orange_count):
        x, z = random_pos()
        spawn("OrangeBall", x, z, f"orangeball_{i}")

