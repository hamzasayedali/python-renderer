import numpy as np
import math
import constants
import helpers
import random
import pygame

import json

class Camera:
    def __init__(self, pos = [34.0,0.0,6.0], direction = 90, pitch = 0, fov=60, up=[0,0,1]):
        # vec3 for point of reference
        self.pos = np.array(pos)
        # rotation about z axis in degrees
        self.direction = direction
        # vec3 for direction the view is facing, normalized direction vector
        self.facing = helpers.normalize(np.array([round(math.cos(math.radians(direction)),5),round(math.sin(math.radians(direction)),5),0]))

        # float for Field of View in radians
        self.fov = fov
        self.up = np.array(up)

        self.is_orbiting = False
        self.orbit_distance = 5

        self.pitch = 0

        self.set_pitch(pitch)

        self.velocity = np.array([0,0,0])

        self.path = [[-12.5,-5.5,9.0],[40.5,-5.5,9.0],[-12.5,-5.5,9.0],[40.5,-5.5,9.0]]
        self.current_path_point = -1
        self.is_following_path = False

        print(f"FACING: {self.facing[0]} {self.facing[1]}")


    def get_perpendicular_axis(self):
        axis = np.cross(self.facing, -self.up)
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
            self.pos = self.pos + np.array([0,0,constants.CAMERA_MOVE_SPEED])

        if direction == "DOWN":
            self.pos = self.pos + np.array([0,0,-constants.CAMERA_MOVE_SPEED])

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


        self.facing = helpers.normalize(np.dot(rot_matrix,self.facing))
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


        self.facing = helpers.normalize(np.dot(rot_matrix,self.facing))

        self.up = np.dot(rot_matrix,self.up)

    def toggle_orbit(self):
        self.is_orbiting = not self.is_orbiting

        if self.is_orbiting:
            
            self.orbit_distance = np.linalg.norm(self.pos - np.array([0,0,0]))

    def toggle_path_follow(self):
        self.is_following_path = not self.is_following_path 
        
        if self.is_following_path:
            self.current_path_point = 0
            self.pos = np.array(self.path[0])

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

    
    def update(self, inputs, orbit_facing = [0,0,0]):
        
        if self.is_orbiting:
            self.orbit([0,0,0], orbit_facing)

        if self.is_following_path:
            self.follow_path()

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

        self.pos += self.velocity
        

    def orbit(self, orbit_point, orbit_facing = [0,0,0]):
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

        direction_to_orbit = self.pos - np.array(orbit_facing)

        angle = helpers.angle_between_vectors_degrees([direction_to_orbit[0],direction_to_orbit[1],0],[1,0,0])

        self.set_direction(angle)

    def follow_path(self):
        next_path_point = self.current_path_point + 1

        if next_path_point == len(self.path):
            next_path_point = 0

       
        direction = helpers.normalize(np.array(self.path[next_path_point]) - self.pos)
        self.pos += direction * constants.CAMERA_MOVE_SPEED

        if np.linalg.norm(self.pos - np.array(self.path[next_path_point])) < 2:
            
            self.pos = np.array(self.path[next_path_point])
            self.current_path_point = next_path_point
        


class Triangle:
    def __init__(self, points=[], base_color = [200, 45,85]):
        self.points = points
        self.color = (random.randint(0,255),random.randint(0,255),random.randint(0,255))

        self.dist_to_camera = 0
        self.middle = (np.array(self.points[0]) + np.array(self.points[1]) + np.array(self.points[2]))/3.0

        self.lighting_is_calculated = False
        self.prev_sun_position = np.array([0,0,0])
        self.adjusted_color = self.color
        self.base_color = base_color

        self.normal_is_calculated = False
        self.normal = [0,0,0]

        self.center_is_calculated = False
        
    
    def update_dist_to_camera(self,camera_coord):
        self.dist_to_camera = np.linalg.norm(self.middle - camera_coord)

    def get_center(self):
        if self.center_is_calculated:
            return self.middle
        
        self.middle = (np.array(self.points[0]) + np.array(self.points[1]) + np.array(self.points[2]))/3.0
        #self.center_is_calculated = True
        return self.middle
    
    def get_normal(self):
        if self.normal_is_calculated:
            return self.normal
        #compute the projection of the normal of the triangle to the direction of the sun. Parallel means full color, perpendicular means duller color
        v1 = np.array(self.points[1]) - np.array(self.points[0])
        v2 = np.array(self.points[2]) - np.array(self.points[0])
    
        # Compute the cross product of the two edge vectors
        normal_vector = np.cross(v1, v2)
        normalized_normal = normal_vector / np.linalg.norm(normal_vector)

        self.normal_is_calculated = True
        self.normal = normalized_normal
        return normalized_normal
    
    def get_lighted_color(self, sun_position, camera_position):

        if (sun_position == self.prev_sun_position).all():
            return self.adjusted_color

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
        self.adjusted_color = adjusted_color
        self.prev_sun_position = sun_position
        return (adjusted_color[0],adjusted_color[1],adjusted_color[2])
    
        #return self.color




class World:
    def __init__(self):
        self.triangles = []
        self.origin = np.array([0,0,0])
        self.sun_direction = np.array([200,100,50])

        self.entities = []

        self.is_sunrising = False
        self.sun_velocity = 10

        self.coins = [Coin([30,-5.5,3])]

        self.load_entities()

        self.score = 0

        

        self.player_pos = np.array([0,0,0])

        

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

        self.update_coins()

        

    
    def update_coins(self):
        for coin in self.coins:
            if np.linalg.norm(coin.pos-self.player_pos) < 3:
                if not coin.claimed:
                    coin.claimed = True
                    self.score += 1
                    print("COIN!!")

    

    def load_entities(self):

        self.triangles = []

        for coin in self.coins:
            if not coin.claimed:
                for triangle in coin.model.mesh_triangles:
                    self.add_triangle(triangle)
        
        for obj in self.entities:
            if isinstance(obj, Floor):
                
                for triangle in obj.triangles:
                    self.add_triangle(triangle)
            if isinstance(obj,Triangle):
                self.add_triangle(obj)

            if isinstance(obj, Prisim):
                for triangle in obj.mesh_triangles:
                    self.add_triangle(triangle)

            if isinstance(obj, Octahedron):
                for triangle in obj.mesh_triangles:
                    self.add_triangle(triangle)

            if isinstance(obj, Tetrahedron):
                for triangle in obj.mesh_triangles:
                    self.add_triangle(triangle)

            if isinstance(obj, Icosahedron):
                for triangle in obj.mesh_triangles:
                    self.add_triangle(triangle)
            if isinstance(obj, ObjFile):
                for triangle in obj.mesh_triangles:
                    self.add_triangle(triangle)

            if isinstance(obj, Robot):
                for triangle in obj.mesh_triangles:
                    self.add_triangle(triangle)

            if isinstance(obj,Player):
                self.player_pos = obj.pos
                for triangle in obj.model.mesh_triangles:
                    self.add_triangle(triangle)

        print(len(self.triangles))




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


    
    def set_pos(self, pos):
        self.pos = pos
        self.generate_mesh

    def compute_perp_axis(self):
        self.perp_axis = np.cross(self.up, self.facing)
        #print(self.perp_axis)

    def contains(self, point):
        u = self.up
        v = self.facing
        w = self.perp_axis

        return True
        
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

        #bottom of prism
        triangle0 = Triangle([b0,b1,b2],base_color=self.color)
        triangle1 = Triangle([b2,b3,b0],base_color=self.color)

        #top of prism
        triangle2 = Triangle([t0,t2,t1],base_color=self.color)
        triangle3 = Triangle([t2,t0,t3],base_color=self.color)


        
        #side 1
        triangle4 = Triangle([b0,t1,b1],base_color=self.color)
        triangle5 = Triangle([t1,b0,t0],base_color=self.color)

        
        #side 2
        triangle6 = Triangle([b2,t3,b3],base_color=self.color)
        triangle7 = Triangle([t3,b2,t2],base_color=self.color)

        
        #side 3
        triangle8 = Triangle([b1,t2,b2],base_color=self.color)
        triangle9 = Triangle([t2,b1,t1],base_color=self.color)

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

class Octahedron:

    def __init__(self, pos=[0,0,0], dim=[1,1,4], up=[0,0,1], facing=[1,0,0], color=[45,200,45]):
        self.pos = np.array(pos)
        self.dim = np.array(dim)

        self.up = np.array(up)
        self.facing = np.array(facing)

        self.perp_axis = np.array([0,0,0])

        self.compute_perp_axis()

        self.color = color

        self.mesh_triangles = []

        self.generate_mesh()


    
    def set_pos(self, pos):
        self.pos = pos
        self.generate_mesh

    def compute_perp_axis(self):
        self.perp_axis = np.cross(self.up, self.facing)
        #print(self.perp_axis)

    def contains(self, point):
        u = self.up
        v = self.facing
        w = self.perp_axis

        return True
        
    def generate_mesh(self):
        # using the position and dimensions, generate a mesh of triangles that form the prisim

        # each prisim needs 12 triangles, 2 per side for 6 sides

        # define corners

        # bottom corners
        b0 = self.pos - self.facing * self.dim[0] / 2.0 - self.perp_axis * self.dim[1] / 2.0 
        b1 = b0 + self.facing * self.dim[0] 
        b2 = b1 + self.perp_axis * self.dim[1]
        b3 = b2 - self.facing * self.dim[0]

        # top point

        u0 = self.pos + self.up * self.dim[2]
        d0 = self.pos - self.up * self.dim[2]

        #face 1
        triangle0 = Triangle([b1,b0,u0],base_color=self.color)
        triangle1 = Triangle([b2,b1,u0],base_color=self.color)

        #top of prism
        triangle2 = Triangle([b3,b2,u0],base_color=self.color)
        triangle3 = Triangle([b0,b3,u0],base_color=self.color)

        #face 1
        triangle4 = Triangle([b0,b1,d0],base_color=self.color)
        triangle5 = Triangle([b1,b2,d0],base_color=self.color)

        #top of prism
        triangle6 = Triangle([b2,b3,d0],base_color=self.color)
        triangle7 = Triangle([b3,b0,d0],base_color=self.color)

        self.mesh_triangles.append(triangle0)
        self.mesh_triangles.append(triangle1)
        self.mesh_triangles.append(triangle2)
        self.mesh_triangles.append(triangle3)
        self.mesh_triangles.append(triangle4)
        self.mesh_triangles.append(triangle5)
        self.mesh_triangles.append(triangle6)
        self.mesh_triangles.append(triangle7)

        print(len(self.mesh_triangles))

class Tetrahedron:

    def __init__(self, pos=[0,0,0], dim=[1,1,4], up=[0,0,1], facing=[1,0,0], color=[45,200,45]):
        self.pos = np.array(pos)
        self.dim = np.array(dim)

        self.up = np.array(up)
        self.facing = np.array(facing)

        self.perp_axis = np.array([0,0,0])

        self.compute_perp_axis()

        self.color = color

        self.mesh_triangles = []

        self.generate_mesh()


    
    def set_pos(self, pos):
        self.pos = pos
        self.generate_mesh

    def compute_perp_axis(self):
        self.perp_axis = np.cross(self.up, self.facing)
        #print(self.perp_axis)

    def contains(self, point):
        u = self.up
        v = self.facing
        w = self.perp_axis

        return True
        
    def generate_mesh(self):
        # using the position and dimensions, generate a mesh of triangles that form the prisim

        # each prisim needs 12 triangles, 2 per side for 6 sides

        # define corners

        # bottom corners

        # points of tetrahedron base at middle of bottom face


        b0 = self.pos + self.facing * self.dim[0] / 2.0
        b1 = self.pos - self.facing * math.cos(math.pi/3) * (self.dim[0] / 2.0) + self.perp_axis * math.sin(math.pi/3) * (self.dim[0] / 2.0)
        b2 = self.pos - self.facing * math.cos(math.pi/3) * (self.dim[0] / 2.0) - self.perp_axis * math.sin(math.pi/3) * (self.dim[0] / 2.0)

        u0 = self.pos + self.up * self.dim[2]

        triangle0 = Triangle([b1,b0,u0],base_color=self.color)
        triangle1 = Triangle([b2,b1,u0],base_color=self.color)

        #top of prism
        triangle2 = Triangle([b0,b2,u0],base_color=self.color)
        triangle3 = Triangle([b0,b1,b2],base_color=self.color)

        self.mesh_triangles.append(triangle0)
        self.mesh_triangles.append(triangle1)
        self.mesh_triangles.append(triangle2)
        self.mesh_triangles.append(triangle3)


class Icosahedron:

    def __init__(self, pos=[0,0,0], dim=[1,1,4], up=[0,0,1], facing=[1,0,0], color=[45,200,45]):
        self.pos = np.array(pos)
        self.dim = np.array(dim)

        self.up = np.array(up)
        self.facing = np.array(facing)

        self.perp_axis = np.array([0,0,0])

        self.compute_perp_axis()

        self.color = color

        self.mesh_triangles = []

        self.generate_mesh()


    
    def set_pos(self, pos):
        self.pos = pos
        self.generate_mesh

    def compute_perp_axis(self):
        self.perp_axis = np.cross(self.up, self.facing)
        #print(self.perp_axis)

    def contains(self, point):
        u = self.up
        v = self.facing
        w = self.perp_axis

        return True
        
    def generate_mesh(self):
        # using the position and dimensions, generate a mesh of triangles that form the prisim

        # each prisim needs 12 triangles, 2 per side for 6 sides

        # define corners

        # bottom corners

        # points of tetrahedron base at middle of bottom face

        triangles = helpers.parse_obj_to_triangles("icosahedron.obj",pos=self.pos,scale=3,invert_mesh=True)

        for triangle in triangles:

            
            triangle0 = Triangle(triangle,base_color=self.color)
            self.mesh_triangles.append(triangle0)


class ObjFile:
    def __init__(self, pos=[0,0,0], dim=[1,1,4], up=[0,0,1], facing=[1,0,0], color=[45,200,45],filename=None, scale=1,invert_mesh=False):
        self.pos = np.array(pos)
        self.dim = np.array(dim)

        self.up = np.array(up)
        self.facing = np.array(facing)

        self.perp_axis = np.array([0,0,0])

        self.compute_perp_axis()

        self.color = color
        self.scale = scale
        self.invert_mesh = invert_mesh
        self.filename = filename

        self.mesh_triangles = []

        self.generate_mesh()


    
    def set_pos(self, pos):
        self.pos = pos
        self.generate_mesh

    def compute_perp_axis(self):
        self.perp_axis = np.cross(self.up, self.facing)
        #print(self.perp_axis)

    def contains(self, point):
        u = self.up
        v = self.facing
        w = self.perp_axis

        return True
        
    def generate_mesh(self):
        # using the position and dimensions, generate a mesh of triangles that form the prisim

        # each prisim needs 12 triangles, 2 per side for 6 sides

        # define corners

        # bottom corners

        # points of tetrahedron base at middle of bottom face

        if self.filename != None:

            triangles = helpers.parse_obj_to_triangles(self.filename,pos=self.pos, scale = self.scale, invert_mesh=self.invert_mesh)

            for triangle in triangles:

                
                triangle0 = Triangle(triangle,base_color=self.color)
                self.mesh_triangles.append(triangle0)

        

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
        self.base_frame = [0,0,1]

        self.L =  [4,5,4,3,4,5,3]
        

        self.theta0 =       [math.pi/4, math.pi/8, math.pi/8, math.pi/2,0,0,0,0]
        
        self.joint_types=   ["R"]
        self.joint_axes=    [[0,0,1],[0,1,0],[0,0,1],[1,0,0],[0,0,1],[0,1,0],[1,0,0]]

        self.up = [0,0,1]
        self.facing = [1,0,0]

        self.current_joint_controlled = 0

        self.mesh_triangles = []

        self.end_effector_pos = [0,0,0]

        self.generate_arm_mesh()

        self.updated = False

        self.wiggling = False

        self.lock_pos = False

        self.wiggling_selected = []
        self.wiggling_direction = []

        self.joint_speed = 0.1

        self.desired_pos = self.theta0.copy()

        for i in range(len(self.L)):
            self.wiggling_selected.append(False)
            self.wiggling_direction.append(random.randint(0,1) * 2 - 1)

        self.control_buttons = []


    def update(self, inputs):
        # max arm speed is ROBOT_ROT_SPEED, min arm speed is 0.5 that, diminises as arm gets longer
        
        if self.wiggling:
            change = random.randint(0,100)
            if change < 10:
                joint = random.randint(0,len(self.L)-1) 
                self.wiggling_selected[joint] = not self.wiggling_selected[joint] 

            for i in range(len(self.L)-1):
                if change % 15 == 0:
                    self.wiggling_direction[i] *= -1
                if self.wiggling_selected[i]:

                    total_length = np.sum(self.L[i:])
                    arm_rot_speed = constants.ROBOT_ROT_SPEED * (0.2 + 0.8 * 1/total_length)

                    
                    self.rotate_joint(i, arm_rot_speed * self.wiggling_direction[i])
                    

            self.updated = True

        elif self.lock_pos:
            self.move_to_desired_joint_theta()

        else:
            total_length = np.sum(self.L[self.current_joint_controlled:])
            arm_rot_speed = constants.ROBOT_ROT_SPEED * (0.3 + 0.7 * 1/(total_length*total_length))

            if inputs["a"]:
                
                self.rotate_joint(self.current_joint_controlled, arm_rot_speed)
                self.updated = True

            if inputs["d"]:
                self.rotate_joint(self.current_joint_controlled, -arm_rot_speed)
                self.updated = True
        

    def rotate_joint(self, joint_index, delta):
        self.theta0[joint_index] += delta
        self.generate_arm_mesh()

    def load_from_json(self, filename):
        f = open(f"./src/gam3r/robot_specs/{filename}", "r")
        x = f.read()
        y = json.loads(x)

        arm_lengths = y["arm_lengths"]
        joint_axes = y["joint_axes"]
        theta0 = y["theta0"]

        self.L = arm_lengths
        self.joint_axes = joint_axes
        self.theta0 = theta0

        self.generate_arm_mesh()
        self.updated = True


    def add_arm(self,L=4,axis=[0,1,0],theta=math.pi/2.0):

        self.L.append(L)
        self.joint_axes.append(axis)
        self.theta0.append(theta)

        self.wiggling_selected.append(False)
        self.wiggling_direction.append(-1)

        self.generate_arm_mesh()
        self.updated = True

    def remove_arm(self):
        self.L.pop()
        self.joint_axes.pop()
        self.theta0.pop()
        self.wiggling_selected.pop()
        self.wiggling_direction.pop()

        self.generate_arm_mesh()
        self.updated = True

    def change_arm_lenth(self, index, new_length = 1):
        self.L[index] = new_length
        self.generate_arm_mesh()
        self.updated = True
    def change_arm_axis(self, index, axis = [1,0,0]):
        self.joint_axes[index] = axis
        self.generate_arm_mesh()
        self.updated = True

    def move_to_desired_joint_theta(self):

        theta_des = self.desired_pos

        edited = False

        for i in range(len(self.theta0)):
            if self.theta0[i] != theta_des[i]:
                edited = True
                if self.theta0[i] - theta_des[i] > 0:
                    self.theta0[i] -= self.joint_speed
                else:
                    self.theta0[i] += self.joint_speed
                
                if abs(self.theta0[i] - theta_des[i]) < self.joint_speed:
                    self.theta0[i] = theta_des[i]
        if edited:
            self.generate_arm_mesh()
            self.updated = True

        else:
            self.lock_pos = False
                

        
    def end_effector_pos_from_theta(self, theta):
        pass

    def ik_from_point(self, point):

        new_theta = self.theta0.copy()
        for i in range(len(new_theta)):
            new_theta[i] = new_theta[i]+0.5
        return new_theta
        

    def generate_arm_mesh(self):

        self.mesh_triangles = []

        arm_color = [212, 198, 49]
        joint_color = [50,50,50]
        joint_controlled_color = [230,230,245]
        end_effector_color = [100,100,200]

        base_pos = self.base_frame
        base_rot = self.up
        base_facing = self.facing

        rotation_matrices = []

        rotations = []
        facings = []
        positions = []

        for axis,theta in zip(self.joint_axes, self.theta0):
            #print(axis," ",theta)
            matrix = get_rot_matrix(axis,theta)
            rotation_matrices.append(matrix)

        for i in range(len(self.L)):
            
            rot_i = np.dot(rotation_matrices[i],base_rot)
            facing_i = np.dot(rotation_matrices[i],base_facing)

            for j in range(i):
                rot_i = np.dot(rotation_matrices[i-(j+1)], rot_i)
                facing_i = np.dot(rotation_matrices[i-(j+1)], facing_i)

            rotations.append(rot_i)
            facings.append(facing_i)

        

        positions.append(base_pos)
        for i in range(len(self.L)):
            positions.append(positions[i] + rotations[i]*self.L[i])

        self.end_effector_pos = positions[-1]
        end_effector_facing = facings[-1]
        end_effector_up = rotations[-1]
        end_effector_prisim_pos = self.end_effector_pos - end_effector_up * 0.5
        end_effector = Prisim(end_effector_prisim_pos,[1.2,1.2,1],up=end_effector_up,facing=end_effector_facing, color = end_effector_color)

        for triangle in end_effector.mesh_triangles:
            self.mesh_triangles.append(triangle)


        for i in range(len(self.L)):
            #build the prisims
            prisim0 = Prisim(positions[i]+rotations[i]*0.6, [1,1,self.L[i]-0.6], up=rotations[i], facing=facings[i], color=arm_color)

            color = joint_color
            if self.current_joint_controlled == i:
                color = joint_controlled_color

            joint0 = Prisim()


            joint0_up = prisim0.facing
            joint0_pos = positions[i] - joint0_up * prisim0.dim[1] / 2.0 
            joint0 = Prisim(joint0_pos, [prisim0.dim[1]*1.2,prisim0.dim[1]*1.2,prisim0.dim[0]],up=joint0_up,facing=prisim0.up, color=color)

            for triangle in prisim0.mesh_triangles:
                self.mesh_triangles.append(triangle)

            for triangle in joint0.mesh_triangles:
                self.mesh_triangles.append(triangle)


    def update_mouse_clicks(self,mouse_pos):
        base_pos = [constants.WIDTH / 2.0 - len(self.L) / 2.0 * 40,0]
        for i in range(len(self.L)):
            #make plus and minus rects
            plus_rect = pygame.Rect(base_pos[0]+40 * i,40,20,20)
            minus_rect = pygame.Rect(base_pos[0]+40 * i + 20,40,20,20)

            if helpers.point_rect_collide(mouse_pos,plus_rect):
                self.L[i] += 1
                self.generate_arm_mesh()
                self.updated = True 
            if helpers.point_rect_collide(mouse_pos,minus_rect) and self.L[i] > 1:
                self.L[i] -= 1
                self.generate_arm_mesh()
                self.updated = True 
            

    def render_control_bar(self, screen, my_font, icons):
        # draw robot select bar
        box_width = 40
        pos = [constants.WIDTH / 2.0 - len(self.L) / 2.0 * box_width,0]


        selected_color = [13,41,63]
        not_selected_color = [1,22,39]

        selected_text = [162,191,252]
        not_selected_text = [137,164,187]

        plus_icon = icons[0]
        minus_icon = icons[1]

        for i in range(len(self.L)):



            if i == self.current_joint_controlled:
                box_color = selected_color
                text_color = selected_text
            else:
                box_color = not_selected_color
                text_color = not_selected_text


            

            pygame.draw.rect(screen,box_color,pygame.Rect(pos[0] + 40 * i,pos[1],40,40))
            pygame.draw.rect(screen,not_selected_color,pygame.Rect(pos[0]+40 * i,40,20,20))
            screen.blit(plus_icon,(pos[0]+40*i,40))
            pygame.draw.rect(screen,not_selected_color,pygame.Rect(pos[0]+40 * i + 20,40,20,20))
            screen.blit(minus_icon,(pos[0]+40*i + 20,40))
            
            text_surface = my_font.render(f'{i+1}', False, text_color)
            text_rect = text_surface.get_rect(center=(pos[0] + 40 * i + box_width/2.0, pos[1] + box_width / 2.0))
            screen.blit(text_surface,text_rect)


    

class Block:
    def __init__(self):
        self.prisim = Prisim([0,0,0],[4,4,4],up=[0,0,1],facing=[1,0,0],color=[150,150,150])

        self.locked_to_arm = True

        self.pos_to_end_effector = [0,0,0]
        
    def update(self, end_effector_pos, hold_is_clicked):
        if self.locked_to_arm:
            self.prisim.set_pos(end_effector_pos + self.pos_to_end_effector)
            self.prisim.mesh_triangles = []
            self.prisim.generate_mesh()

        
            



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

class PointGridPlane:
    def __init__(self):

        self.plane = np.array([0,0,1])
        self.facing = np.array([1,0,0])
        self.step_size = 1
        self.grid_radius = 3
        self.pos = np.array([0,0,0])

        self.points = []
        self.generate_points()
    
    def generate_points(self):

        self.side_axis = np.cross(self.plane,self.facing)

        for i in range(self.grid_radius * 2 + 1):
            offset = self.grid_radius

            for j in range(self.grid_radius * 2 + 1):

                self.points.append(self.pos + self.side_axis * (j-offset) + self.facing * (i-offset))

class MenuBackground:

    def __init__(self):
        self.pos = (0,0)

        self.img = pygame.image.load('./assets/menu_background/bg.png')
        self.img = pygame.transform.scale(self.img, (constants.WIDTH, constants.HEIGHT))

        self.foreground = pygame.image.load('./assets/menu_background/far-buildings.png')
        self.foreground = pygame.transform.scale(self.foreground, (constants.WIDTH, constants.HEIGHT))

        self.nearground = pygame.image.load('./assets/menu_background/buildings.png')
        self.nearground = pygame.transform.scale(self.nearground, (constants.WIDTH, constants.HEIGHT))

        self.buildings = pygame.image.load('./assets/menu_background/skill-foreground.png')
        self.buildings = pygame.transform.scale(self.buildings, (constants.WIDTH, constants.HEIGHT))

    def render(self, screen):

        screen.blit(self.img, (0,0))
        screen.blit(self.foreground, (0,0))
        screen.blit(self.nearground, (0,0))
        screen.blit(self.buildings, (0,0,))


    

def default_onclick():
    print("Default onclick")
class MenuButton:

    def __init__(self, text = "sample text", pos = (200,100), height=40, on_click = default_onclick, align = "center") -> None:
        
        self.text = text
        self.padding = 10
        self.height = 40

        self.pos = pos

        self.color = (100,100,100)
        self.text_color = (255,255,255)

        self.align = align

        self.hovered = False

        self.rect = pygame.Rect(0,0,0,0)

        self.on_click = on_click



    def render(self, screen,my_font):
        
        text_surface = my_font.render(f' {self.text} ', False, self.text_color)
        if self.align == "center":
            text_rect = text_surface.get_rect(center=(self.pos[0],self.pos[1]))
        if self.align == "left":
            text_rect = text_surface.get_rect(midleft=(self.pos[0],self.pos[1]))
        self.rect = text_rect
        pygame.draw.rect(screen,self.color,text_rect)
        screen.blit(text_surface,text_rect)

    def is_hovered(self, pos) -> bool:
        if pos[0] >= self.rect.left and pos[0] < self.rect.left + self.rect.width and pos[1] >= self.rect.top and pos[1] < self.rect.top + self.rect.height:
            self.color = (150,150,150)
            self.hovered = True
            return True
        self.color = (100,100,100)
        self.hovered = False
        return False
    
    def update_mouse_clicks(self,pos):
        
        if self.hovered:

            self.on_click()
            
            return True
        return False



class TextBox:
    def __init__(self,text = "sample text", pos = (200,100), height=40):
        self.text = text
        self.padding = 10
        self.height = 40

        self.pos = pos

        self.color = (100,100,100)
        self.text_color = (255,255,255)

        self.rect = pygame.Rect(0,0,0,0)

    def render(self, screen,my_font):
        
        text_surface = my_font.render(f' {self.text} ', False, self.text_color)
        text_rect = text_surface.get_rect(center=(self.pos[0],self.pos[1]))
        self.rect = text_rect
        pygame.draw.rect(screen,self.color,text_rect)
        screen.blit(text_surface,text_rect)


def hallway(pos):

    triangles = []

    prisims = []
    prisims.append(Prisim(pos,[4,0.3,4],[0,0,1],[1,0,0]))
    pos2 = np.array(pos)+np.array([0,5,0])
    prisims.append(Prisim([pos2[0],pos2[1],pos2[2]],[4,0.3,4],[0,0,1],[1,0,0]))

    for prisim in prisims:
        for triangle in prisim.mesh_triangles:
            triangles.append(triangle)


    return triangles


class Floor:
    def __init__(self, pos,num_tiles,color= [84, 54, 99]):
        triangles = []
    
        size = 4

        

        for x in range(num_tiles):
            for y in range(num_tiles):
                

                tile_pos = np.array([pos]) + np.array([x,y,0]) * size


                point1 = np.ndarray.tolist(tile_pos)[0]
                point2 = np.ndarray.tolist(tile_pos + size * np.array([1,0,0]))[0]
                point3 = np.ndarray.tolist(tile_pos + size * np.array([1,1,0]))[0]
                point4 = np.ndarray.tolist(tile_pos + size * np.array([0,1,0]))[0]
                
                triangle1 = Triangle([point1,point3,point2],color)
                triangle2 = Triangle([point3, point1, point4],color)

                triangles.append(triangle1)
                triangles.append(triangle2)
                triangle1 = Triangle([point1,point2,point3],color)
                triangle2 = Triangle([point3, point4, point1],color)

                triangles.append(triangle1)
                triangles.append(triangle2)

        self.triangles = triangles

        print(len(self.triangles))

def floor(pos,num_tiles,color= [84, 54, 99]):

    triangles = []
    
    size = 4

     

    for x in range(num_tiles):
        for y in range(num_tiles):
            

            tile_pos = np.array([pos]) + np.array([x,y,0]) * size


            point1 = np.ndarray.tolist(tile_pos)[0]
            point2 = np.ndarray.tolist(tile_pos + size * np.array([1,0,0]))[0]
            point3 = np.ndarray.tolist(tile_pos + size * np.array([1,1,0]))[0]
            point4 = np.ndarray.tolist(tile_pos + size * np.array([0,1,0]))[0]
            
            triangle1 = Triangle([point1,point2,point3],color)
            triangle2 = Triangle([point3, point4, point1],color)

            triangles.append(triangle1)
            triangles.append(triangle2)

            
   


    return triangles

class Boat:
    def __init__(self,pos=[0,0,0]):
        self.pos = pos
        self.facing = [1,0,0]
        self.up = [0,0,1]


        self.mesh_triangles = []
        self.generate_mesh()

    def generate_mesh(self):
        
        self.mesh_triangles = []

        #5 brown prisims as logs

        log_width = 1
        log_length = 7

        back = np.array(self.pos) - np.array(self.facing) * (log_length / 2)
        side_axis = np.cross(np.array(self.facing),np.array(self.up))

        for i in range(5):

            log_pos = back + side_axis * (i - 2)*1.1
            prisim = Prisim([log_pos[0],log_pos[1],log_pos[2]], [log_width,log_width,log_length],self.facing,self.up,color=[79, 60, 20])

            for triangle in prisim.mesh_triangles:
                self.mesh_triangles.append(triangle)





class ArmControlGui:
    def __init__(self,index):
        self.index = index
        self.pos = (constants.WIDTH - 200,index * 50)
        self.grow_button = MenuButton("+",self.pos,40)

    def update(self, mouse_pos, mouse_click):

        pass
            

class Player:
    def __init__(self,pos):
        self.pos = pos
        self.model = ObjFile(self.pos,[6,2,4],helpers.normalize(np.array([0,0,1])),[1,0,0],[200,200,200],"humanoid_tri.obj",scale=0.3,invert_mesh=True)
    
    def update_pos(self, pos):
        self.pos = pos
        self.model = ObjFile(self.pos,[6,2,4],helpers.normalize(np.array([0,0,1])),[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)],"humanoid_tri.obj",scale=0.5,invert_mesh=True)

class Coin:
    def __init__(self,pos):
        self.pos = pos
        self.model = Octahedron(pos, [2,1,1],[0,0,1],[1,0,0],[255,200,0])
        self.claimed = False
        

            


