

class Pose:
    """
    a data structure that support get/set the information of a car
    """
    def __init__(self, position, heading, curvature, speed, acceleration, jerk):
        self.position = position
        self.heading = heading
        self.curvature = curvature
        self.speed = speed
        self.acceleration = acceleration
        self.jerk = jerk

    def to_dict(self):
        return {
            'position': self.position,
            'heading': self.heading,
            'curvature': self.curvature,
            'speed': self.speed,
            'acceleration': self.acceleration,
            'jerk': self.jerk
        }