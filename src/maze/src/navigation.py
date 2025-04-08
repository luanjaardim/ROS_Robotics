#!/usr/bin/env python3
import rospy as rp
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from map import generate_map_matrix, plt_map
from enum import Enum
from drivers.srv import DetectDirServ, DetectDirServRequest

class State(Enum):
    # First stage of the puzzle, while has wall at both sides
    CORRIDOR = 0
    # End of the corridor, end of corridor walls, will walk a little further before changing to the next state
    END_CORRIDOR = 1
    # Take a picture of both sides, and then turn to the choosen one
    CAMERA_DECISION = 2
    # Go ahead while it's possible
    WALK_TO_DEST = 3

class Navigator:
    def __init__(self, grid_size=101, resolution = 0.1, wall_tolerance=4):
        rp.init_node('navigator_node')
        self.grid_size = grid_size
        self.resolution = resolution
        self.wall_tolerance = wall_tolerance
        self.state = State.CORRIDOR
        self.vel_pub = rp.Publisher('/cmd_vel', Twist, queue_size=1)
        self.laser_sub = rp.Subscriber('/LaserScan', LaserScan, self.get_lidar_msg)
        # ldm -> left distances mean, rdm -> right distances mean, lc -> left count, rc -> right count
        self.ldm, self.rdm, self.lc, self.rc = (1e12, 1e12, 0, 0)

    def get_lidar_msg(self, msg):
        grid, points, angles = generate_map_matrix(msg.ranges, msg.angle_min, msg.angle_max, grid_size=self.grid_size, resolution=self.resolution)
        # plt_map(grid, points, angles, msg.ranges, grid_size=self.grid_size)

        if self.state == State.CORRIDOR:
            dist_left, dist_right = (0, 0)

            found = False
            for i in range(1, self.grid_size//2):
                for j in range(-3, 4):
                    if grid[j + self.grid_size//2][i + self.grid_size//2] == 1:
                        dist_right = i
                        found = True
                        break
                if found:
                    break
            # is_wall = False
            # if not np.isinf(dist_right):
            #     for k in [-1, 0, 1]:
            #         if grid[j+(self.grid_size//2)-1][dist_right + k + self.grid_size//2] == 1:
            #             is_wall = True 
            #             break
            # if not is_wall:
            #     self.wall_not_found_counter += 1
            # else:
            #     self.wall_not_found_counter = 0

            found = False
            for i in range(-1, -self.grid_size//2, -1):
                for j in range(-3, 4):
                    if grid[j + self.grid_size//2][i + self.grid_size//2] == 1:
                        dist_left = -i
                        found = True
                        break
                if found:
                    break
            # is_wall = False
            # if not np.isinf(dist_left):
            #     for k in [-1, 0, 1]:
            #         if grid[j+(self.grid_size//2)-1][dist_left + k + self.grid_size//2] == 1:
            #             is_wall = True 
            #             break
            # if not is_wall:
            #     self.wall_not_found_counter += 1
            # else:
            #     self.wall_not_found_counter = 0
            
            # Send calculated speed to motors, using distance to calculate the desired speed for each wheel
            vel_ang = max(min((dist_left - dist_right) * 60 // 2, 100), -100)
            vel = Twist(linear=Vector3(x=40.0), angular=Vector3(z=vel_ang))
            self.vel_pub.publish(vel)

            if dist_right-self.rdm > self.wall_tolerance or dist_left-self.ldm > self.wall_tolerance:
                self.state = State.END_CORRIDOR
            else:
                # Update previous distance mean
                if dist_left != 0:
                    self.lc += 1
                    self.ldm = ((self.ldm * (self.lc - 1) + dist_left)/ self.lc)
                if dist_right != 0:
                    self.rc += 1
                    self.rdm = ((self.rdm * (self.rc - 1) + dist_right)/ self.rc)

            print(f"ang_vel: {vel_ang}, dist_l: {dist_left}, dist_r: {dist_right}, l_mean: {round(self.ldm, 4)}, r_mean: {round(self.rdm, 4)}")
            # print(target_x, target_y)
            # TODO: Check if grid[self.grid_size//2][self.grid_size//2+dist] can reach the ones below it, if not we lost the wall

        elif self.state == State.END_CORRIDOR:
            print('END CORRIDOR STATE')
            # Walk forward for a brief time
            self.state = State.CAMERA_DECISION

        elif self.state == State.CAMERA_DECISION:
            print('DETECT SHAPES')
            vel = Twist(linear=Vector3(x=60.0))
            self.vel_pub.publish(vel)
            rp.wait_for_service('/detect_shapes')
            try:
                from time import sleep
                detect_service = rp.ServiceProxy('/detect_shapes', DetectDirServ)
                msg = detect_service()
                print(f"msg {msg.dir.data}")
                if msg.dir.data == 'right':
                    # Turn right 90 degrees
                    vel = Twist(angular=Vector3(z=-850.0))
                    self.vel_pub.publish(vel)
                    sleep(0.5)
                    self.state = State.WALK_TO_DEST
                elif msg.dir.data == 'left':
                    # Turn left 90 degrees
                    vel = Twist(angular=Vector3(z=850.0))
                    self.vel_pub.publish(vel)
                    sleep(0.5)
                    self.state = State.WALK_TO_DEST
                else:
                    rp.logerr("Did not find any valid shape")
                    exit(1)
            except rp.ServiceException as e:
                rp.logerr("Service call failed: %s", e)
        
        elif self.state == State.WALK_TO_DEST:
            # Walk to the received direction while possible
            # if dist > 3:
            vel = Twist(linear=Vector3(x=60.0))
            self.vel_pub.publish(vel)
            # pass

if __name__ == "__main__":
    Navigator(grid_size=101, resolution=0.05)
    rp.spin()