import numpy as np
import math
import constants
import helpers
import random

class Camera:
    def __init__(self, pos = [3.0,0.0,0.0], direction = 180, pitch = 0, fov=60, up=[0,0,-1]):
        # vec3 for point of reference
        self.pos = np.array(pos)
        # rotation about z axis in degrees
        self.direction = direction
        # vec3 for direction the view is facing, normalized direction vector
        self.facing = np.array([round(math.cos(math.radians(direction)),5),round(math.sin(math.radians(direction)),5),0])

        # float for Field of View in radians
        self.fov = fov
        self.up = np.array(up)

        self.is_orbiting = False
        self.orbit_distance = 5

        self.pitch = pitch

        print(f"FACING: {self.facing[0]} {self.facing[1]}")

    def get_perpendicular_axis(self):
        axis = np.cross(self.facing, self.up)
        axis = axis / np.linalg.norm(axis)
        return axis

    def move(self, direction):
        if direction == "FORWARD":
            movement_vector = self.facing/np.linalg.norm(self.facing) * constants.CAMERA_MOVE_SPEED

            self.pos += np.array(movement_vector)
        
        if direction == "BACKWARD":
            movement_vector = self.facing/np.linalg.norm(self.facing) * constants.CAMERA_MOVE_SPEED

            self.pos -= np.array(movement_vector)

        if direction == "UP":
            self.pos = self.pos + np.array([0,0,-constants.CAMERA_MOVE_SPEED])

        if direction == "DOWN":
            self.pos = self.pos + np.array([0,0,+constants.CAMERA_MOVE_SPEED])

        if direction == "LEFT":
            self.pos = self.pos + self.get_perpendicular_axis() * (-constants.CAMERA_MOVE_SPEED)
        
        if direction == "RIGHT":
            self.pos = self.pos + self.get_perpendicular_axis() * (constants.CAMERA_MOVE_SPEED)

    def set_direction(self, degrees):

        delta_direction = degrees - self.direction
        self.direction = degrees

        theta_rad = math.radians(delta_direction)

        u = np.array([0,0,1])

        a = np.cos(theta_rad / 2.0)
        b, c, d = - u * np.sin(theta_rad / 2.0)

        aa,bb,cc,dd = a*a, b*b, c*c, d*d

        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d

        rot_matrix = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


        self.facing = np.dot(rot_matrix,self.facing)
        self.up = np.dot(rot_matrix,self.up)

        #self.facing = np.array([round(math.cos(math.radians(self.direction)),5),round(math.sin(math.radians(self.direction)),5),self.facing[2]])

    def set_pitch(self, degrees):

        delta_pitch = degrees - self.pitch
        self.pitch = degrees

        ## rotate facing and up by the required tilt

        tilt_theta_rad = math.radians(delta_pitch)

        print("THETA: ")
        # get the vector cross product between the up direction and the 

        tilt_axis = np.cross(self.facing, self.up)

        print(F"Tilt axis: {tilt_axis}")

        u = tilt_axis / np.linalg.norm(tilt_axis)
        u = np.asfarray(u)

        print(F"Tilt axis normalized: {u}")

        

        # make rotation matrix about this axis

        a = np.cos(tilt_theta_rad / 2.0)
        b, c, d = - u * np.sin(tilt_theta_rad / 2.0)

        aa,bb,cc,dd = a*a, b*b, c*c, d*d

        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d

        rot_matrix = np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


        self.facing = np.dot(rot_matrix,self.facing)

        self.up = np.dot(rot_matrix,self.up)

    def toggle_orbit(self):
        self.is_orbiting = not self.is_orbiting

        if self.is_orbiting:
            self.orbit_distance = np.linalg.norm(self.pos - np.array([0,0,0]))

    def turn(self, direction):
        if direction == "LEFT":
            self.set_direction(self.direction - constants.TURN_SPEED)
            print("turn left")
            
        if direction == "RIGHT":
            self.set_direction(self.direction + constants.TURN_SPEED)
            print("turn right")

    def tilt(self, direction):
        if direction == "UP":
            self.set_pitch(self.pitch + 10)

        if direction == "DOWN":
            self.set_pitch(self.pitch - 10)

    
    def update(self):
        
        if self.is_orbiting:
            self.orbit([0,0,0])

    def orbit(self, orbit_point):
        # face the origin

        # do the camera movement

        pos_in_xy0 = np.array([self.pos[0],self.pos[1],0])
        #print(f"POS IN XY TO START {pos_in_xy0}")

        theta = math.pi / 60
        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0],
                           [math.sin(theta), math.cos(theta), 0],
                           [0,0,1]])
        
        #print(rotation_matrix)
        
        new_pos_in_xy = np.matmul(pos_in_xy0, rotation_matrix)

        #print(new_pos_in_xy)

        self.pos = np.array([new_pos_in_xy[0], new_pos_in_xy[1], self.pos[2]])
        #print(f"POSITION: {self.pos} END")

        direction_to_orbit = self.pos - np.array(orbit_point)

        angle = helpers.angle_between_vectors_degrees([direction_to_orbit[0],direction_to_orbit[1],0],[1,0,0])

        self.set_direction(angle)


class Triangle:
    def __init__(self, points=[], base_color = [200, 45,85]):
        self.points = points
        self.color = (random.randint(0,255),random.randint(0,255),random.randint(0,255))

        self.dist_to_camera = 0
        self.middle = (np.array(self.points[0]) + np.array(self.points[1]) + np.array(self.points[2]))/3.0


        self.base_color = base_color
    
    def update_dist_to_camera(self,camera_coord):
        self.dist_to_camera = np.linalg.norm(self.middle - camera_coord)

    def get_center(self):
        self.middle = (np.array(self.points[0]) + np.array(self.points[1]) + np.array(self.points[2]))/3.0
        return self.middle
    
    def get_lighted_color(self, sun_position, camera_position):

        #figure out if I'm between the triangle and the sun

        sun_direction = self.get_center() - sun_position
        
        #compute the projection of the normal of the triangle to the direction of the sun. Parallel means full color, perpendicular means duller color
        v1 = np.array(self.points[1]) - np.array(self.points[0])
        v2 = np.array(self.points[2]) - np.array(self.points[0])
    
        # Compute the cross product of the two edge vectors
        normal_vector = np.cross(v1, v2)
        normalized_normal = normal_vector / np.linalg.norm(normal_vector)

        
        normalized_sun = sun_direction/np.linalg.norm(sun_direction)
        

        camera_to_object = self.get_center() - camera_position

        light_side = np.dot(camera_to_object,normalized_normal) > 0

        light_add = 1
        if not light_side:
            light_add = -1

        light_level = ((np.dot(normalized_sun,normalized_normal) * light_add + 1) / 2) * 0.9 + 0.1

        

        adjusted_color = np.array(self.base_color) * light_level
        return (adjusted_color[0],adjusted_color[1],adjusted_color[2])
    
        #return self.color




class World:
    def __init__(self):
        self.triangles = []
        self.origin = np.array([0,0,0])
        self.sun_direction = np.array([10,100,-10])

        self.is_sunrising = False
        self.sun_velocity = 10

    def add_triangle(self, triangle):
        self.triangles.append(triangle)

    def toggle_sunrising(self):
        self.is_sunrising = not self.is_sunrising

    def update(self):
        if self.is_sunrising:
            self.sun_direction = np.array([
                self.sun_direction[0],
                self.sun_direction[1],
                self.sun_direction[2] + self.sun_velocity])
            
            if abs(self.sun_direction[2]) > 100:
                self.sun_velocity *= -1



