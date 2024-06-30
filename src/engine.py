import pygame
import time
import numpy as np
import random
import math

import helpers

import cube_model

# Constants
WIDTH = 640
HEIGHT = 640

CAMERA_MOVE_SPEED = 0.5
TURN_SPEED = 5


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

        print(f"FACING: {self.facing[0]} {self.facing[1]}")

    def move(self, direction):
        if direction == "FORWARD":
            movement_vector = self.facing/np.linalg.norm(self.facing) * CAMERA_MOVE_SPEED

            self.pos += np.array(movement_vector)
        
        if direction == "BACKWARD":
            movement_vector = self.facing/np.linalg.norm(self.facing) * CAMERA_MOVE_SPEED

            self.pos -= np.array(movement_vector)

    def set_direction(self, degrees):
        self.direction = degrees
        self.facing = np.array([round(math.cos(math.radians(self.direction)),5),round(math.sin(math.radians(self.direction)),5),0])

    def toggle_orbit(self):
        self.is_orbiting = not self.is_orbiting

        if self.is_orbiting:
            self.orbit_distance = np.linalg.norm(self.pos - np.array([0,0,0]))

        
            

    def turn(self, direction):
        if direction == "LEFT":
            self.set_direction(self.direction - TURN_SPEED)
            print("turn left")
            
        if direction == "RIGHT":
            self.set_direction(self.direction + TURN_SPEED)
            print("turn right")

    
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
    def __init__(self, points=[], base_color = [45, 200,85]):
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


class Engine:

    def __init__(self, name):
        self.name = name
        self.screen = None

        self.world = World()
        self.camera = Camera()

        #list of points to draw after rendering
        self.render_result = []


    def populate_world(self):
        
        # test objects to add to the world
        # triangle located at the origin in then y-z plane
        
        
        triangle2 = Triangle([
            (0,0,-2),
            (-1,1,0),
            (-1,-1,0)])
        triangle3 = Triangle([
            (-1,-1,0),
            (1,-1,0),
            (0,0,-2)])
        triangle4 = Triangle([
            (0,0,-2),
            (1,1,0),
            (1,-1,0)])
        triangle5 = Triangle([
            (0,0,2),
            (-1,1,0),
            (-1,-1,0)])
        triangle6 = Triangle([
            (-1,-1,0),
            (1,-1,0),
            (0,0,2)])
        triangle7 = Triangle([
            (0,0,2),
            (1,1,0),
            (1,-1,0)])
        triangle8 = Triangle([
            (0,0,2),
            (1,1,0),
            (-1,1,0)])
        
        
        
        
        self.world.add_triangle(triangle2)
        self.world.add_triangle(triangle3)
        self.world.add_triangle(triangle4)
        self.world.add_triangle(triangle5)
        self.world.add_triangle(triangle6)
        self.world.add_triangle(triangle7)
        self.world.add_triangle(triangle8)

        for triangle in cube_model.cube_model:
            self.world.add_triangle(Triangle(triangle))
        
        


    def startup(self):
        print("startup")
        try:
            #Initalize Pygame
            pygame.init()
            #Create Window with custom title
            pygame.display.set_caption(self.name)
            self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
            pygame.display.flip()
            

        except Exception as e:
            print(e)
            exit()

        # startup functions.
        self.populate_world()

    

    def inputs(self):
        #print("Accept input")
        pass

    def update(self):
        #print("Update game")

        # adding camera path movement

        self.camera.update()
        self.world.update()
        pass

    def flatten_triangle(self, triangle):
        #function takes a triangle with a list of 3d points and outputs a list of 2d screen coordinates


        output = []
        for point in triangle.points:
            output.append((point[1],point[2]))

        return output
    
    def project_vector(self,a, b):
        # Calculate the dot product of a and b
        dot_product_ab = np.dot(a, b)
        # Calculate the dot product of b with itself
        dot_product_bb = np.dot(b, b)
        # Calculate the projection
        projection = (dot_product_ab / dot_product_bb) * b
        return projection
    
    def find_camera_rel_point(self, point):

        camera_to_point = self.camera.pos - point

        normal_to_plane_of_point = self.project_vector(camera_to_point, self.camera.facing)

        deviation = normal_to_plane_of_point - camera_to_point

        # get screen x and y values
        y_component = self.project_vector(deviation, self.camera.up)

        x_component = deviation - y_component

        direction_of_x = np.dot(x_component,np.cross(self.camera.facing,self.camera.up)) > 0

        final_x_component = np.linalg.norm(x_component)

        if direction_of_x == False:
            final_x_component = final_x_component * -1

        direction_of_y = np.dot(y_component, self.camera.up) > 0

        final_y_component = np.linalg.norm(y_component)

        if direction_of_y == True:
            final_y_component = final_y_component * -1

        

        non_scaled_point = np.array([final_x_component,final_y_component])

        distance_of_plane = np.linalg.norm(normal_to_plane_of_point)
        scale_factor = 1
        if distance_of_plane != 0:
            scale_factor = (0.75/distance_of_plane) * WIDTH


        scaled_point = non_scaled_point * scale_factor

        return scaled_point

    def render(self):
        print("Render scene")

        # need to process the triangles in the world and figure out
        # where to draw them relative to the camera

        self.screen.fill((255,255,255))


        # render origin point

        origin = self.world.origin

        vec_to_origin = self.camera.pos - origin

        normal_to_plane_of_point = self.project_vector(vec_to_origin, self.camera.facing)

        deviation = normal_to_plane_of_point - vec_to_origin


        print(vec_to_origin)
        print(normal_to_plane_of_point)
        print(deviation)

        non_scaled_point = np.array([deviation[1],deviation[2]])

        distance_of_plane = np.linalg.norm(normal_to_plane_of_point)

        scale_factor = (1/distance_of_plane) * WIDTH

        scaled_point = self.find_camera_rel_point(origin)

        move_to_center_screen = np.array([[WIDTH/2.0, HEIGHT/2.0]])
        
        
        final_point = (scaled_point + move_to_center_screen)[0]
        
        pygame.draw.circle(self.screen,(255,0,0),(final_point[0],final_point[1]),10)

        for triangle in self.world.triangles:
            triangle.update_dist_to_camera(self.camera.pos)

        # sort triangles by distance to camera
        depth_buffer_triangles = sorted(self.world.triangles, key=lambda x: x.dist_to_camera, reverse=True)

        for triangle in depth_buffer_triangles:

            new_points = []

            for point in triangle.points:
                new_point = self.find_camera_rel_point(point)

                new_point = (new_point + move_to_center_screen)[0]

                new_points.append(new_point)

            # get color of triangle
            lighted_color = triangle.get_lighted_color(self.world.sun_direction,self.camera.pos)
            
            pygame.draw.polygon(self.screen,lighted_color,new_points)

            triangle_center = triangle.get_center()

            new_center = (self.find_camera_rel_point(triangle_center) + move_to_center_screen)[0]

            #pygame.draw.circle(self.screen,(0,255,0),(new_center[0],new_center[1]),1)


        pygame.draw.rect(self.screen,(0,0,0),pygame.Rect(WIDTH/2-1,HEIGHT/2-10,2,20))
        pygame.draw.rect(self.screen,(0,0,0),pygame.Rect(WIDTH/2-10,HEIGHT/2-1,20,2))

        


    def run(self):
        print(self.name)

        self.startup()

        while True:

            try:
                

                self.inputs()
                self.update()
                self.render()

                pygame.display.update()
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        break

                    # Check for key presses
                    elif event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_UP:
                            # Handle up key press
                            print("Up arrow key pressed")
                            self.camera.pos = self.camera.pos + np.array([0,0,-CAMERA_MOVE_SPEED])
                        elif event.key == pygame.K_DOWN:
                            # Handle down key press
                            print("Down arrow key pressed")
                            self.camera.pos = self.camera.pos + np.array([0,0,CAMERA_MOVE_SPEED])
                        elif event.key == pygame.K_LEFT:
                            # Handle left key press
                            print("Left arrow key pressed")
                            self.camera.pos = self.camera.pos + np.array([0,CAMERA_MOVE_SPEED,0])
                        elif event.key == pygame.K_RIGHT:
                            # Handle right key press
                            print("Right arrow key pressed")
                            self.camera.pos = self.camera.pos + np.array([0,-CAMERA_MOVE_SPEED,0])
                        elif event.key == pygame.K_PERIOD:
                            # Handle left key press
                            print("Period key pressed")
                            self.camera.move("FORWARD")
                            #self.camera.pos = self.camera.pos + np.array([-CAMERA_MOVE_SPEED,0,0])
                        elif event.key == pygame.K_COMMA:
                            # Handle right key press
                            print("Comma key pressed")
                            self.camera.move("BACKWARD")
                            #self.camera.pos = self.camera.pos + np.array([CAMERA_MOVE_SPEED,0,0])
                        elif event.key == pygame.K_SEMICOLON:
                            # Handle left key press
                            print("Semicolon key pressed")
                            self.camera.turn("LEFT")
                        elif event.key == pygame.K_QUOTE:
                            # Handle right key press
                            print("Quote key pressed")
                            self.camera.turn("RIGHT")
                        elif event.key == pygame.K_SPACE:
                            self.camera.toggle_orbit()
                        elif event.key == pygame.K_s:
                            self.world.toggle_sunrising()


                
            except KeyboardInterrupt:
                
                pygame.quit()
                
                break

            time.sleep(0.05)

