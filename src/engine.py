import pygame
import time
import numpy as np
import random
import math

# Constants
WIDTH = 640
HEIGHT = 640

CAMERA_MOVE_SPEED = 0.5
TURN_SPEED = 5


class Camera:
    def __init__(self, pos = [3,1,1], direction = 180, pitch = 0, fov=60, up=[0,0,1]):
        # vec3 for point of reference
        self.pos = np.array(pos)
        # rotation about z axis in degrees
        self.direction = direction
        # vec3 for direction the view is facing, normalized direction vector
        self.facing = np.array([round(math.cos(math.radians(direction)),2),round(math.sin(math.radians(direction)),2),0])

        
        # float for Field of View in radians
        self.fov = fov
        self.up = np.array(up)

        print(f"FACING: {self.facing[0]} {self.facing[1]}")

    def turn(self, direction):
        if direction == "LEFT":
            self.direction = self.direction - TURN_SPEED
            print("turn left")
            
        if direction == "RIGHT":
            self.direction = self.direction + TURN_SPEED
            print("turn right")

        self.facing = np.array([round(math.cos(math.radians(self.direction)),2),round(math.sin(math.radians(self.direction)),2),0])
        


class Triangle:
    def __init__(self, points=[]):
        self.points = points
        self.color = (random.randint(0,255),random.randint(0,255),random.randint(0,255))

class World:
    def __init__(self):
        self.triangles = []
        self.origin = np.array([0,0,0])

    def add_triangle(self, triangle):
        self.triangles.append(triangle)

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
        triangle20 = Triangle([
            (-5,0,-2),
            (-5,1,0),
            (-5,-1,0)])
        
        triangle30 = Triangle([
            (-5,2,-2),
            (-5,3,0),
            (-5,1,0)])
        triangle40 = Triangle([
            (-5,-2,-2),
            (-5,-1,0),
            (-5,-3,0)])
        
        triangle2 = Triangle([
            (-1,0,-2),
            (-1,1,0),
            (-1,-1,0)])
        
        triangle3 = Triangle([
            (-1,2,-2),
            (-1,3,0),
            (-1,1,0)])
        triangle4 = Triangle([
            (-1,-2,-2),
            (-1,-1,0),
            (-1,-3,0)])
        
        triangle = Triangle([
            (0,0,-2),
            (0,1,0),
            (0,-1,0)])
        
        triangle = Triangle([
            (2,-3,0),
            (3,-3,-2),
            (4,-3,0)])
        
        self.world.add_triangle(triangle20)
        self.world.add_triangle(triangle30)
        self.world.add_triangle(triangle40)
        self.world.add_triangle(triangle2)
        self.world.add_triangle(triangle3)
        self.world.add_triangle(triangle4)
        self.world.add_triangle(triangle)
        


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

        direction_of_y = np.dot(y_component, self.camera.up)

        final_y_component = np.linalg.norm(y_component)

        if direction_of_y == False:
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

            new_points = []

            for point in triangle.points:
                new_point = self.find_camera_rel_point(point)

                new_point = (new_point + move_to_center_screen)[0]

                new_points.append(new_point)

            
            pygame.draw.polygon(self.screen,triangle.color,new_points)


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
                            self.camera.pos = self.camera.pos + np.array([0,-CAMERA_MOVE_SPEED,0])
                        elif event.key == pygame.K_RIGHT:
                            # Handle right key press
                            print("Right arrow key pressed")
                            self.camera.pos = self.camera.pos + np.array([0,CAMERA_MOVE_SPEED,0])
                        elif event.key == pygame.K_PERIOD:
                            # Handle left key press
                            print("Period key pressed")
                            self.camera.pos = self.camera.pos + np.array([-CAMERA_MOVE_SPEED,0,0])
                        elif event.key == pygame.K_COMMA:
                            # Handle right key press
                            print("Comma key pressed")
                            self.camera.pos = self.camera.pos + np.array([CAMERA_MOVE_SPEED,0,0])
                        elif event.key == pygame.K_SEMICOLON:
                            # Handle left key press
                            print("Semicolon key pressed")
                            self.camera.turn("LEFT")
                        elif event.key == pygame.K_QUOTE:
                            # Handle right key press
                            print("Quote key pressed")
                            self.camera.turn("RIGHT")


                
            except KeyboardInterrupt:
                
                pygame.quit()
                
                break

            time.sleep(0.1)

