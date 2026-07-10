import math


def _within(p, c, radius):
    """True if 2D points p and c are within `radius` (ignores any z)."""
    return math.hypot(p[0] - c[0], p[1] - c[1]) < radius


class BallCollector:
    def __init__(self, supervisor, sensor_name="collection_mechanism"):
        self.supervisor = supervisor
        timestep = int(supervisor.getBasicTimeStep())
        self.sensor = supervisor.getDevice(sensor_name)
        self.sensor.enable(timestep)
        self.robot_node = supervisor.getSelf()
        self.score = 0
        self.collected_balls = []          # world (x, y, z) of collected balls

    def check_and_collect(self, collect_radius=0.3):
        """If the sensor is pressed and a ball is in range, collect the nearest
        one, score it, remove it, and return its position. Otherwise None."""
        if self.sensor.getValue() <= 0:
            return None

        children = self.supervisor.getRoot().getField("children")
        robot_pos = self.robot_node.getPosition()

        closest, closest_type, min_dist = None, None, float("inf")
        for i in range(children.getCount()):
            node = children.getMFNode(i)
            if node is None:
                continue
            node_type = node.getTypeName()
            if node_type in ("OrangeBall", "MetalBall"):
                d = math.dist(robot_pos, node.getPosition())   # 3D distance
                if d < min_dist:
                    closest, closest_type, min_dist = node, node_type, d

        if closest is None or min_dist >= collect_radius:
            return None

        pos = closest.getPosition()        # capture BEFORE removing the node
        if closest_type == "OrangeBall":
            self.score += 3
        elif closest_type == "MetalBall":
            self.score += 4
        closest.remove()
        self.collected_balls.append(pos)
        return pos

    def ball_pos_without_collected(self, positions, radius=0.15):
        """Return a new list with any tracked position near a collected ball
        removed. Non-mutating and safe on empty input."""
        return [p for p in positions
                if not any(_within(p, c, radius) for c in self.collected_balls)]
