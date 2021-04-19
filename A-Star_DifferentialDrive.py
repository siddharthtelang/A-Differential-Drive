from Obstacle_new import *
from Visualization_new import *
import numpy as np
import cv2
import matplotlib.pyplot as plt

obs = Obstacle(0.5)
viz = Visualization(obs)
print('Radius in viz = ' + str(viz.getRadius()))

space_size = [10, 10]
space_map = np.zeros([space_size[0], space_size[1], 3], dtype=np.uint8)
#space_map = updateMap(space_map, init_node, [0,0,255])
viz.addObstacles2Map(space_map)
plt.imshow(space_map)
plt.show()