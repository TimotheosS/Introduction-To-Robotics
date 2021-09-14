#!/usr/bin/env python3

# Python standard library
from math import cos, sin, atan2, fabs

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        """
        Fill in your solution here
        """
        x_max = 0
        x_min = grid_map.get_width()

        y_max = 0
        y_min = grid_map.get_height()

        obs = []
        free_spa = []

        counter = 0
        print('\n\n\n')

        for angle in np.arange(scan.angle_min, scan.angle_max + scan.angle_increment, scan.angle_increment):
            print(angle)
            if((scan.ranges[counter] <= scan.range_min) or (scan.ranges[counter] >= scan.range_max)):
                d = 0 # Dummy line
            else:
                # Convert the laser scan ranges and bearings to coordinates in the laser frame
                x_scanned = scan.ranges[counter] * cos(angle)  # Coordinates according to Robot frame
                y_scanned = scan.ranges[counter] * sin(angle) 

                # Scanned to Robot
                x_robot = x_scanned * cos(robot_yaw) - y_scanned * sin(robot_yaw)
                y_robot = y_scanned * cos(robot_yaw) + x_scanned * sin(robot_yaw)

                # Convert the coordinates to the map frame
                x_total = x_robot - origin.position.x  + pose.pose.position.x
                y_total = y_robot - origin.position.y  + pose.pose.position.y

                # Convert the coordinates to map indices
                x_map = int(x_total / resolution)
                y_map = int(y_total / resolution)

                obs.append([x_map,y_map])

                x_init = int((pose.pose.position.x - origin.position.x) / resolution)
                y_init = int((pose.pose.position.y - origin.position.y) / resolution)

                xy_start = (x_init , y_init)
                xy_end = (x_map , y_map)

                empty_space = self.raytrace(xy_start , xy_end)

                for cell in empty_space:
                    free_spa.append([cell[0],cell[1]])
                
                # Fill in the occupied cells
                # self.add_to_map(grid_map , x_map , y_map , self.occupied_space)

                if (x_map < x_min):
                    x_min = x_map
                if (x_map > x_max):
                    x_max = x_map
                if (y_map < y_min):
                    y_min = y_map
                if (y_map > y_max):
                    y_max = y_map


            counter += 1

        for cell in free_spa:
            self.add_to_map(grid_map , cell[0] , cell[1] , self.free_space)

        for cell in obs:
            self.add_to_map(grid_map , cell[0] , cell[1] , self.occupied_space)


        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = x_min
        # The minimum y index in 'grid_map' that has been updated
        update.y = y_min
        # Maximum x index - minimum x index + 1
        update.width = x_max - x_min + 1
        # Maximum y index - minimum y index + 1
        update.height = y_max - y_min + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []

        for x in range(update.x , update.x + update.width):
            for y in range(update.y , update.y + update.height):
                    update.data.append(grid_map.__getitem__([x,y]))

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = 0
        # The minimum y index in 'grid_map' that has been updated
        update.y = 0
        # Maximum x index - minimum x index + 1
        update.width = 0
        # Maximum y index - minimum y index + 1
        update.height = 0
        # The map data inside the rectangle, in row-major order.
        update.data = []

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def inflate_map(self, grid_map):
        """
        Fill in your solution here
        """

        x_max = grid_map.get_width()
        y_max = grid_map.get_height()

        for x in range(x_max):
            for y in range(y_max):
                if(grid_map[x,y] == self.occupied_space):
                    for x_red in range(-self.radius, self.radius + 1):
                        for y_red in range(-self.radius, self.radius + 1):
                            if(grid_map[x + x_red , y + y_red] != self.occupied_space) and (x_red ** 2 + y_red ** 2 <= self.radius ** 2):
                               self.add_to_map(grid_map , x + x_red , y + y_red , self.c_space) 
        
        # Return the inflated map
        return grid_map
