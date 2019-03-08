#!/usr/bin/env python

import numpy as np
from turtle.turtle_formation import TurtleFormation
import time


def main():
    neighbors = np.array([
        [0, 0, 0],
	[1, 0, 1],
	[1, 1, 0],
    ])
    relative_pos = np.array([
        [-0.5, -0.5],  # 2 relative to 1
        [-1, 0],  # 2 relative to 3
	[0.5, -0.5],  # 3 relative to 1
	[1, 0],  # 3 relative to 2
    ])
    path = [
        [5, 0],
	[4, 0],
	[4, -1],
	[3.5, -1.5],
	[3, -1.5],
	[0.5, -1.5],
	# [-0.5, -1],
	# [-1.0, -0.75],
	# [-1.5, -0.5],
	[-2, 0],
	[-1, 3],
	[0, 4],
	[5, 0],
    ]
    fc = TurtleFormation(
        numAgent=3,
	ledr=0, 
	handle=0.2,
	gain=0.4,
	nei=neighbors,
	displ=relative_pos,
	way=path,
	navigation='cam')
    fc.move()
    # time.sleep(10)
    # print("Changing connection Graph!")
    # neighbors2 = np.asarray([[0, 0, 0],[0, 0, 1],[1, 1, 0]])
    # displacement2 = np.asarray([[-1, 0],[0.5, -0.5], [1, 0]])
    # fc.changeConn(neighbors2, displacement2)
    time.sleep(100)
    fc.stop()
    fc.plot('all','all')


if __name__ == "__main__":
    main()
