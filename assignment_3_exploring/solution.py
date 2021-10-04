#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# {Timotheos Souroulla}
# {timsou@kth.se}

from dubins import *
import random
from math import pi, atan2, copysign


class Node():

    def __init__(self,x=None,y=None,theta=None,phi=None,parent=None):
        # Each node data and initialization to none
        self.x = x
        self.y = y
        self.theta = theta
        self.phi = phi
        self.parent = parent
        return

def check_bounds(x,y,xlb, xub, ylb, yub):
    # Return True if out of bounds
    if (x < xlb + 0.2) or (x > xub - 0.2) or (y < ylb + 0.2) or (y > yub - 0.2):
        return True
    else:
        # Return False if not out of bounds
        return False

def obstacle_collide(x, y, car):

    for obst in car.obs :
        # Return True if in obstacle
        if ((obst[1] - y) ** 2 + (obst[0] - x) ** 2) <= (obst[2] + 0.3) ** 2:
            return True
        # Return False if not in obstacle
    return False

def solution(car):

    ''' <<< write your code below >>> '''
    Explored = [ Node(car.x0, car.y0, 0) ]

    # Initialise the 2 lists returned
    times=[]
    controls=[]

    # Initialize step and a counter
    dt = 0.35
    c1 = 0

    while c1 < 8000:
        
        c1 = c1 + 1
        
        # Create a new random node
        if (Explored[-1].x - car.xt)**2 + (Explored[-1].y - car.yt)**2 > 2:
            x_current = random.uniform(car.xlb, car.xub)
            y_current = random.uniform(car.ylb + 0.2 , car.yub - 0.2)
            theta_current = random.uniform(-pi, pi)

            # Optimization trick
            if len(Explored) < 4000:
                ft = Explored
            else:
                ft = Explored[-4000:-1]

            # Create new nodes until not in obstacle or out of bounds
            while (obstacle_collide(x_current, y_current, car)):
                x_current = random.uniform(car.xlb, car.xub)
                y_current = random.uniform(car.ylb + 0.2 , car.yub - 0.2)            

            # The new Node
            toExplore = Explored[-1]

            # The new node's distance
            min_dist = (toExplore.y - y_current) ** 2 + (toExplore.x - x_current) ** 2 + (toExplore.theta - theta_current) ** 2

            for q in ft:

                dist = (q.y - y_current) ** 2 + (q.x - x_current) ** 2 + (q.theta - theta_current) ** 2

                if dist <= min_dist:
                    min_dist = dist
                    toExplore = q

            # Update phi
            phi = atan2(y_current - toExplore.y, x_current - toExplore.x) - toExplore.theta 

            if abs(phi) > pi/4:
                phi = copysign(pi/4, phi)

            # Get next step
            x_next, y_next, theta_next = step(car, toExplore.x, toExplore.y, toExplore.theta, phi, dt)

            # If there is no collision save to tree
            if (obstacle_collide(x_next, y_next, car) == False) and not(check_bounds(x_next, y_next, car.xlb, car.xub, car.ylb, car.yub)):
                Explored.append(Node(x_next, y_next, theta_next, phi, toExplore))

            continue
        else:
            break 

    best_path = [Explored[-1]]
    
    while True:
        q = best_path[-1].parent
        if q != None:
            best_path.append(q)
        else:
            break

    for counter in range(2 , len(best_path) + 1):
        controls.append(best_path[-counter].phi)
    for counter in range(0 , len(controls) + 1):
        times.append(counter * dt)
        
    times = tuple(times)

    return controls, times
