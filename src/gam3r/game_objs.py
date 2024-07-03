import numpy as np
import math
import constants
import helpers
import random
import pygame

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
            self.set_pitch(self.pitch + constants.TURN_SPEED)

        if direction == "DOWN":
            self.set_pitch(self.pitch - constants.TURN_SPEED)

    
    def update(self, inputs):
        
        if self.is_orbiting:
            self.orbit([-5,-5,-3])


        # call functions based on current input status

        if inputs["UP"]:
            self.move("UP")
            pass
        if inputs["DOWN"]:
            self.move("DOWN")
            pass

        if inputs["LEFT"]:
            self.move("LEFT")
            pass
        if inputs["RIGHT"]:
            self.move("RIGHT")
            pass
        if inputs["COMMA"]:
            self.move("BACKWARD")
            pass
        if inputs["PERIOD"]:
            self.move("FORWARD")
            pass
        if inputs["SEMICOLON"]:
            self.turn("LEFT")
            pass
        if inputs["QUOTE"]:
            self.turn("RIGHT")
            pass
        if inputs["LEFTBRACKET"]:
            self.tilt("UP")
            pass
        if inputs["RIGHTBRACKET"]:
            self.tilt("DOWN")
            pass
        

    def orbit(self, orbit_point):
        # face the origin

        # do the camera movement

        pos_in_xy0 = np.array([self.pos[0],self.pos[1],0])
        #print(f"POS IN XY TO START {pos_in_xy0}")

        theta = math.pi / constants.ORBIT_SPEED

        rotation_matrix = np.array([[math.cos(theta), -math.sin(theta), 0],
                           [math.sin(theta), math.cos(theta), 0],
                           [0,0,1]])
        
        #print(rotation_matrix)

        shift_to_origin = pos_in_xy0 - np.array([orbit_point[0], orbit_point[1], 0])
        
        new_pos_in_xy = np.matmul(shift_to_origin, rotation_matrix)

        new_pos_in_xy = new_pos_in_xy + (pos_in_xy0 - shift_to_origin)

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



class Prisim:

    def __init__(self, pos=[0,0,0], dim=[1,1,4], up=[0,0,-1], facing=[1,0,0], color=[45,200,45]):
        self.pos = np.array(pos)
        self.dim = np.array(dim)

        self.up = np.array(up)
        self.facing = np.array(facing)

        self.perp_axis = np.array([0,0,0])

        self.compute_perp_axis()

        self.color = color

        self.mesh_triangles = []

        self.generate_mesh()


    


    def compute_perp_axis(self):
        self.perp_axis = np.cross(self.up, self.facing)
        #print(self.perp_axis)

    
        
    def generate_mesh(self):
        # using the position and dimensions, generate a mesh of triangles that form the prisim

        # each prisim needs 12 triangles, 2 per side for 6 sides

        # define corners

        # bottom corners
        b0 = self.pos - self.facing * self.dim[0] / 2.0 - self.perp_axis * self.dim[1] / 2.0 
        b1 = b0 + self.facing * self.dim[0] 
        b2 = b1 + self.perp_axis * self.dim[1]
        b3 = b2 - self.facing * self.dim[0]

        t0 = b0 + self.up * self.dim[2]
        t1 = b1 + self.up * self.dim[2]
        t2 = b2 + self.up * self.dim[2]
        t3 = b3 + self.up * self.dim[2]

        triangle0 = Triangle([b0,b1,b2],base_color=self.color)
        triangle1 = Triangle([b2,b3,b0],base_color=self.color)
        triangle2 = Triangle([t0,t1,t2],base_color=self.color)
        triangle3 = Triangle([t2,t3,t0],base_color=self.color)

        triangle4 = Triangle([b0,b1,t1],base_color=self.color)
        triangle5 = Triangle([t1,t0,b0],base_color=self.color)

        triangle6 = Triangle([b2,b3,t3],base_color=self.color)
        triangle7 = Triangle([t3,t2,b2],base_color=self.color)

        triangle8 = Triangle([b1,b2,t2],base_color=self.color)
        triangle9 = Triangle([t2,t1,b1],base_color=self.color)

        triangle10 = Triangle([b0,b3,t3],base_color=self.color)
        triangle11 = Triangle([t3,t0,b0],base_color=self.color)

        self.mesh_triangles.append(triangle0)
        self.mesh_triangles.append(triangle1)
        self.mesh_triangles.append(triangle2)
        self.mesh_triangles.append(triangle3)
        self.mesh_triangles.append(triangle4)
        self.mesh_triangles.append(triangle5)
        self.mesh_triangles.append(triangle6)
        self.mesh_triangles.append(triangle7)
        self.mesh_triangles.append(triangle8)
        self.mesh_triangles.append(triangle9)
        self.mesh_triangles.append(triangle10)
        self.mesh_triangles.append(triangle11)

def get_rot_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / np.linalg.norm(axis)
    ux, uy, uz = axis
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    one_minus_cos_theta = 1 - cos_theta
    
    return np.array([
        [cos_theta + ux**2 * one_minus_cos_theta, ux * uy * one_minus_cos_theta - uz * sin_theta, ux * uz * one_minus_cos_theta + uy * sin_theta],
        [uy * ux * one_minus_cos_theta + uz * sin_theta, cos_theta + uy**2 * one_minus_cos_theta, uy * uz * one_minus_cos_theta - ux * sin_theta],
        [uz * ux * one_minus_cos_theta - uy * sin_theta, uz * uy * one_minus_cos_theta + ux * sin_theta, cos_theta + uz**2 * one_minus_cos_theta]
    ])

class Robot:

    def __init__(self):

        self.base_frame = [-5,-5,-3]

        self.L =  [4,5,6,2]
        self.theta0 =       [math.pi/4, math.pi/8, math.pi/8, math.pi/2]
        self.joint_types=   ["R"]
        self.joint_axes=    [[0,0,1],[0,1,0],[0,0,1],[1,0,0]]

        self.mesh_triangles = []

        

        self.generate_arm_mesh()

        self.current_joint_controlled = 0

        self.updated = False

    def update(self, inputs):

        # max arm speed is ROBOT_ROT_SPEED, min arm speed is 0.5 that, diminises as arm gets longer
        total_length = np.sum(self.L[self.current_joint_controlled:])
        arm_rot_speed = constants.ROBOT_ROT_SPEED * (0.3 + 0.7 * 1/total_length)
        if inputs["a"]:
            
            self.rotate_joint(self.current_joint_controlled, arm_rot_speed)
            self.updated = True

        if inputs["d"]:
            self.rotate_joint(self.current_joint_controlled, -arm_rot_speed)
            self.updated = True



    def rotate_joint(self, joint_index, delta):

        self.theta0[joint_index] += delta
        self.generate_arm_mesh()

    def generate_arm_mesh(self):

        self.mesh_triangles = []
        
        pos_0 = self.base_frame
        rot_0 = [0,0,-1]

        facing_0 = [1,0,0]

        rot_0 = np.dot(get_rot_matrix(self.joint_axes[0],self.theta0[0]), rot_0)
        facing_0 = np.dot(get_rot_matrix(self.joint_axes[0],self.theta0[0]), facing_0)
        #rotate and then translate

        pos_1_0 = rot_0 * self.L[0] 

        rot_1 = [0,0,-1]
        facing_1 = [1,0,0]

        rot_1 = np.dot(get_rot_matrix(self.joint_axes[0],self.theta0[0]), np.dot(get_rot_matrix(self.joint_axes[1],self.theta0[1]), rot_1))
        facing_1 = np.dot(get_rot_matrix(self.joint_axes[0],self.theta0[0]),np.dot(get_rot_matrix(self.joint_axes[1],self.theta0[1]), facing_1))

        pos_2_1 = rot_1 * self.L[1]

        rot_2 = [0,0,-1]
        facing_2 = [1,0,0]

        rot_2 = np.dot(get_rot_matrix(self.joint_axes[0],self.theta0[0]), np.dot(get_rot_matrix(self.joint_axes[1],self.theta0[1]), np.dot(get_rot_matrix(self.joint_axes[2],self.theta0[2]),rot_2)))
        facing_2 = np.dot(get_rot_matrix(self.joint_axes[0],self.theta0[0]), np.dot(get_rot_matrix(self.joint_axes[1],self.theta0[1]), np.dot(get_rot_matrix(self.joint_axes[2],self.theta0[2]), facing_2)))
        
        pos_3_2 = rot_2 * self.L[2]

        rot_3 = [0,0,-1]
        facing_3 = [1,0,0]

        rot_3 = np.dot(get_rot_matrix(self.joint_axes[0],self.theta0[0]), np.dot(get_rot_matrix(self.joint_axes[1],self.theta0[1]), np.dot(get_rot_matrix(self.joint_axes[2],self.theta0[2]),np.dot(get_rot_matrix(self.joint_axes[3],self.theta0[3]),rot_3))))
        facing_3 = np.dot(get_rot_matrix(self.joint_axes[0],self.theta0[0]), np.dot(get_rot_matrix(self.joint_axes[1],self.theta0[1]), np.dot(get_rot_matrix(self.joint_axes[2],self.theta0[2]),np.dot(get_rot_matrix(self.joint_axes[3],self.theta0[3]),facing_3))))
        

        prisim0 = Prisim(pos_0, [1,1,self.L[0]], up=rot_0, facing=facing_0)

        base0_up = prisim0.facing
        base0_pos = prisim0.pos - base0_up * prisim0.dim[1] / 2.0 
        base0 = Prisim(base0_pos, [prisim0.dim[1]*1.2,prisim0.dim[1]*1.2,prisim0.dim[0]],up=base0_up,facing=prisim0.up, color=[50,50,50])

        prisim1 = Prisim(pos_0 + pos_1_0, [0.8,0.8,self.L[1]],up=rot_1,facing=facing_1)

        base1_up = prisim1.facing
        base1_pos = prisim1.pos - base1_up * prisim1.dim[1] / 2.0 
        base1 = Prisim(base1_pos, [prisim1.dim[1]*1.2,prisim1.dim[1]*1.2,prisim1.dim[0]],up=base1_up,facing=prisim1.up, color=[50,50,50])


        prisim2 = Prisim(pos_0 + pos_1_0 + pos_2_1, [0.6,0.6,self.L[2]],up=rot_2,facing=facing_2)
        base2_up = prisim2.facing
        base2_pos = prisim2.pos - base2_up * prisim2.dim[1] / 2.0 
        base2 = Prisim(base2_pos, [prisim2.dim[1]*1.2,prisim2.dim[1]*1.2,prisim2.dim[0]],up=base2_up,facing=prisim2.up, color=[50,50,50])

        prisim3 = Prisim(pos_0 + pos_1_0 + pos_2_1 + pos_3_2, [0.4,0.4,self.L[3]],up=rot_3,facing=facing_3)
        base3_up = prisim3.facing
        base3_pos = prisim3.pos - base3_up * prisim3.dim[1] / 2.0 
        base3 = Prisim(base3_pos, [prisim3.dim[1]*1.2,prisim3.dim[1]*1.2,prisim3.dim[0]],up=base3_up,facing=prisim3.up, color=[50,50,50])


        for triangle in prisim0.mesh_triangles:
            self.mesh_triangles.append(triangle)

        for triangle in base0.mesh_triangles:
            self.mesh_triangles.append(triangle)
        
        for triangle in prisim1.mesh_triangles:
            self.mesh_triangles.append(triangle)

        for triangle in base1.mesh_triangles:
            self.mesh_triangles.append(triangle)

        for triangle in prisim2.mesh_triangles:
            self.mesh_triangles.append(triangle)

        for triangle in base2.mesh_triangles:
            self.mesh_triangles.append(triangle)
        
        for triangle in prisim3.mesh_triangles:
            self.mesh_triangles.append(triangle)

        for triangle in base3.mesh_triangles:
            self.mesh_triangles.append(triangle)
    


class Skybox:
    def __init__(self):
        self.pos_z = 1000

        self.color = [200,200,200]

        self.mesh_triangles = []

        self.generate_triangle_mesh()

    def generate_triangle_mesh(self):

        world_size = 10000

        c0, c1, c2, c3 = [-world_size,-world_size,self.pos_z], [world_size,-world_size,self.pos_z],[world_size,world_size,self.pos_z],[-world_size,world_size,self.pos_z]

        triangle0 = Triangle([c0,c1,c2],self.color)
        triangle1 = Triangle([c2,c3,c0],[200,200,240])

        self.mesh_triangles = [triangle0,triangle1]

            


