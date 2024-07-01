import pygame
import time
import numpy as np

import models as models

from game_objs import Camera, World, Triangle

import constants

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
        
        octahedron = models.octahedron()

        cube = models.cube()

        for triangle in octahedron:
            self.world.add_triangle(triangle)

        for triangle in cube:
            self.world.add_triangle(Triangle(triangle))
        
    def startup(self):
        print("startup")
        try:
            #Initalize Pygame
            pygame.init()
            #Create Window with custom title
            pygame.display.set_caption(self.name)
            self.screen = pygame.display.set_mode((constants.WIDTH, constants.HEIGHT))
            pygame.display.flip()
        
        except Exception as e:
            print(e)
            exit()

        # startup functions.
        self.populate_world()


    def inputs(self, event):
        if event.key == pygame.K_UP:
            # Handle up key press
            self.camera.move("UP")
            
        elif event.key == pygame.K_DOWN:
            # Handle down key press
            self.camera.move("DOWN")
            #self.camera.pos = self.camera.pos + np.array([0,0,constants.CAMERA_MOVE_SPEED])
        elif event.key == pygame.K_LEFT:
            # Handle left key press
            self.camera.move("LEFT")
            #self.camera.pos = self.camera.pos + np.array([0,constants.CAMERA_MOVE_SPEED,0])
        elif event.key == pygame.K_RIGHT:
            # Handle right key press
            self.camera.move("RIGHT")
            #self.camera.pos = self.camera.pos + np.array([0,-constants.CAMERA_MOVE_SPEED,0])
        elif event.key == pygame.K_PERIOD:
            # Handle left key press
            self.camera.move("FORWARD")
            
        elif event.key == pygame.K_COMMA:
            # Handle right key press
            self.camera.move("BACKWARD")
            
        elif event.key == pygame.K_SEMICOLON:
            # Handle left key press
            self.camera.turn("LEFT")
        elif event.key == pygame.K_QUOTE:
            # Handle right key press
            self.camera.turn("RIGHT")
        elif event.key == pygame.K_SPACE:
            self.camera.toggle_orbit()
        elif event.key == pygame.K_s:
            self.world.toggle_sunrising()
        elif event.key == pygame.K_RIGHTBRACKET:
            self.camera.tilt("DOWN")
        elif event.key == pygame.K_LEFTBRACKET:
            self.camera.tilt("UP")


    def update(self):
        
        self.camera.update()
        self.world.update()
        

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
            scale_factor = (0.75/distance_of_plane) * constants.WIDTH


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

        scale_factor = (1/distance_of_plane) * constants.WIDTH

        scaled_point = self.find_camera_rel_point(origin)

        move_to_center_screen = np.array([[constants.WIDTH/2.0, constants.HEIGHT/2.0]])
        
        
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


        pygame.draw.rect(self.screen,(0,0,0),pygame.Rect(constants.WIDTH/2-1,constants.HEIGHT/2-10,2,20))
        pygame.draw.rect(self.screen,(0,0,0),pygame.Rect(constants.WIDTH/2-10,constants.HEIGHT/2-1,20,2))

        


    def run(self):
        print(self.name)

        # initializing
        self.startup()

        # game loop
        while True:

            try:
                #input handling
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        break
                    
                    # Check for key presses
                    elif event.type == pygame.KEYDOWN:
                        self.inputs(event)

                # update game state
                self.update()

                # draw game state to screen
                self.render()

                pygame.display.update()

            except KeyboardInterrupt:
                
                pygame.quit()
                
                break

            time.sleep(0.05)

