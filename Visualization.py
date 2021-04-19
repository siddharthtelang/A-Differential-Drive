class Visualization:
    def __init__(self, obstacle):
        self.radius = obstacle.robot_radius
        self.obstacle = obstacle

    def getRadius(self):
        return self.radius

    def updateMapViz(self, space_map, state, color):
        X, Y, _ = space_map.shape
        transformed_y = state[0]
        transformed_x = X - state[1] -1
        space_map[transformed_x, transformed_y, :] = color

    def addObstacles2Map(self, map):
        #circles
        x_offset = self.obstacle.circle_x_offset
        y_offset = self.obstacle.circle_y_offset_1
        radius = self.obstacle.circle_radius
        for i in range(x_offset - radius, x_offset + radius):
            for j in range(y_offset - radius, y_offset + radius):
                if (i - x_offset) **2 + (j - y_offset)**2 <= radius**2:
                    self.updateMapViz(map, [i, j], [255, 0, 0])
