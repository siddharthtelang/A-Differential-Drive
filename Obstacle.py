class Obstacle:
    def __init__(self, clearance):
        self.x = 10
        self.y = 10
        self.clearance = 0
        self.robot_radius = 0.4 + self.clearance
        self.circle_radius = 1
        self.circle_x_offset = 2
        self.circle_y_offset_1 = 2
        self.circle_y_offset_2 = 8
