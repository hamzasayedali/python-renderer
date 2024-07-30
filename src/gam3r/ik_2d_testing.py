import pygame
import time
import numpy as np
import math
import helpers

import models as models

from game_objs import Camera, World, Triangle, Prisim, Robot, Skybox, PointGridPlane, MenuButton, MenuBackground, hallway, floor, Boat, ArmControlGui

from render_funcs import CameraDetails
import constants




L = [5,3]
theta = [0,0]
num_joints = 2

base_pos = (0,0)

arms_to_render = []

for i in range(num_joints):

    












class Engine:

    def __init__(self, name):
        self.name = name
        self.screen = None


    def populate_world(self):
        
        pass

        
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

        pass


        


    def update(self):

       pass
        

    def draw_point(self, point):
        color = (0,0,150)
        pos = np.array(point) * -1 + np.array([constants.WIDTH/2.0,constants.HEIGHT/2.0])
        
        pygame.draw.circle(self.screen, color,pos,10)
    
    def render(self):

        # need to process the triangles in the world and figure out
        # where to draw them relative to the camera

        self.screen.fill((255,255,255))

        self.draw_point((0,0))



    def run(self):

        clock = pygame.time.Clock()
        print(self.name)

        # initializing
        self.startup()

        # game loop
        while True:
            start_time = time.time() # start time of the loop

            try:
                #input handling
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        break
                    
                    # Check for key presses
                    elif event.type == pygame.KEYDOWN or event.type == pygame.KEYUP or event.type == pygame.MOUSEBUTTONDOWN:
                        self.inputs(event)

                

                # update game state
                self.update()

                # draw game state to screen
                self.render()

                pygame.display.update()

            except KeyboardInterrupt:
                
                pygame.quit()
                
                break

            time.sleep(0.005)
            #clock.tick(60)
            fps = 1.0 / (time.time() - start_time)
            pygame.display.set_caption(f"FPS: {round(fps,2)}")


engine = Engine("2d IK")

engine.run()