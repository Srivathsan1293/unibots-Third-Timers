import math

class IMU:
    def __init__(self, robot, name="inertial unit", timestep=None,
                 yaw_offset=0.0, yaw_sign=1.0):
        self.unit = robot.getDevice(name)
        if self.unit is None:
            raise RuntimeError(
                f"No InertialUnit named '{name}'. Add one to the robot and "
                f"match its 'name' field, or pass the correct name.")
        ts = timestep if timestep is not None else int(robot.getBasicTimeStep())
        self.unit.enable(ts)
        self.yaw_offset = yaw_offset
        self.yaw_sign = yaw_sign

    def roll_pitch_yaw(self):
        """Raw [roll, pitch, yaw] in radians, world-referenced."""
        return self.unit.getRollPitchYaw()

    def yaw(self):
        """Heading in radians, wrapped to (-pi, pi], with offset/sign applied so
        it matches the +x->+y convention used by the tag localizer."""
        raw = self.unit.getRollPitchYaw()[2]
        y = self.yaw_sign * raw + self.yaw_offset
        return math.atan2(math.sin(y), math.cos(y))
