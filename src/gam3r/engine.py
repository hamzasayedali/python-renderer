import pygame
import time
import numpy as np
import math
import helpers

import models as models

from game_objs import Camera, World, Triangle, Prisim, Robot, Skybox, PointGridPlane, MenuButton, MenuBackground, hallway, floor, Boat, ArmControlGui, Block

from render_funcs import CameraDetails
import constants

class Engine:

    def __init__(self, name):
        self.name = name
        self.screen = None

        self.current_view = "GAME"

        self.world = World()
        self.camera = Camera()

        self.robot = Robot()
        self.block = Block()
        self.current_robot_joint = 0

        self.grid = PointGridPlane()

        self.display_developer_info = False

        self.fps = 0

        #list of points to draw after rendering
        self.render_result = []

        self.inputs_dict = {
            "LEFT": False,
            "RIGHT": False,
            "UP": False,
            "DOWN": False,
            "COMMA": False,
            "PERIOD": False,
            "SEMICOLON": False,
            "QUOTE": False,
            "LEFTBRACKET": False,
            "RIGHTBRACKET": False,
            "a": False,
            "d": False,
        }

        self.menu_button = MenuButton("Start Game", (200,200))
        self.math_button = MenuButton("See the Math", (200,250))
        self.menu_background = MenuBackground()

        self.game_background = pygame.image.load('./assets/game_background.jpg')
        self.game_background = pygame.transform.scale(self.game_background, (constants.WIDTH, constants.HEIGHT))

        self.plus_icon = pygame.image.load('./assets/plus.png')
        self.minus_icon = pygame.image.load('./assets/minus.png')



    def populate_world(self):
        
        # test objects to add to the world
        # triangle located at the origin in then y-z plane
        
        octahedron = models.octahedron()

        cube = models.cube()

        

        for triangle in self.block.prisim.mesh_triangles:
            self.world.add_triangle(triangle)
        

        

        #for triangle in octahedron:
        #    self.world.add_triangle(triangle)

        for triangle in cube:
            self.world.add_triangle(Triangle(triangle))


        prisim = Prisim([-6,3,0],[1,1,4])
        prisim2 = Prisim([-6,3,-4],[4,1,1])

        #for triangle in prisim.mesh_triangles:
        #    self.world.add_triangle(triangle)

        #for triangle in prisim2.mesh_triangles:
        #    self.world.add_triangle(triangle)

        for triangle in self.robot.mesh_triangles:
            self.world.add_triangle(triangle)

        
        #skybox = Skybox()
        #for triangle in skybox.mesh_triangles:
        #    self.world.add_triangle(triangle)

        
        triangle = Triangle(
            [[0,2,0],[0,-2,0],[0,0,-2]],[150,150,200]
            )
        triangle2 = Triangle(
            [[8,2,0],[8,-2,0],[8,0,-2]],[250,150,150]
            )
        
        triangle3 = Triangle(
            [[2,4,0],[6,4,0],[4,4,-2]],[250,150,150]
            )
        
        triangle4 = Triangle(
            [[2,-4,0],[6,-4,0],[4,-4,-2]],[255,100,150]
            )
        
        triangle5 = Triangle([
            [1000,0,1000],
            [-1000,-1000,1000],
            [-1000,1000,1000]
        ])
        
        #self.world.triangles = []


        hall = hallway([0,10,0])

        game_floor = floor([-16,-16,-1],8)

        #for triangle in hall:
        #    self.world.add_triangle(triangle)

        for triangle in game_floor:
            self.world.add_triangle(triangle)
        
        self.world.add_triangle(triangle)


        
    def startup(self):
        print("startup")
        try:
            #Initalize Pygame
            pygame.init()
            #Create Window with custom title
            pygame.display.set_caption(self.name)
            self.screen = pygame.display.set_mode((constants.WIDTH, constants.HEIGHT))
            pygame.display.flip()

            pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.
            self.my_font = pygame.font.SysFont('Comic Sans MS', 30)
        
        except Exception as e:
            print(e)
            exit()

        # startup functions.
        self.populate_world()


    def inputs(self, event):

        keypress = False

        

        if event.type == pygame.MOUSEBUTTONDOWN:
            
            mouse_pos = pygame.mouse.get_pos()
            self.robot.update_mouse_clicks(mouse_pos)
            return

            

        if event.type == pygame.KEYDOWN:
            keypress = True

        if event.key == pygame.K_UP:
            # Handle up key press
            #self.camera.move("UP")
            self.inputs_dict["UP"] = keypress

        elif event.key == pygame.K_DOWN:
            # Handle down key press
            #self.camera.move("DOWN")
            self.inputs_dict["DOWN"] = keypress
            #self.camera.pos = self.camera.pos + np.array([0,0,constants.CAMERA_MOVE_SPEED])
        elif event.key == pygame.K_LEFT:
            # Handle left key press
            #self.camera.move("LEFT")
            self.inputs_dict["LEFT"] = keypress
            #self.camera.pos = self.camera.pos + np.array([0,constants.CAMERA_MOVE_SPEED,0])
        elif event.key == pygame.K_RIGHT:
            # Handle right key press
            #self.camera.move("RIGHT")
            self.inputs_dict["RIGHT"] = keypress
            #self.camera.pos = self.camera.pos + np.array([0,-constants.CAMERA_MOVE_SPEED,0])
        elif event.key == pygame.K_PERIOD:
            # Handle left key press
            #self.camera.move("FORWARD")
            self.inputs_dict["PERIOD"] = keypress
            
        elif event.key == pygame.K_COMMA:
            # Handle right key press
            #self.camera.move("BACKWARD")
            self.inputs_dict["COMMA"] = keypress
            
        elif event.key == pygame.K_SEMICOLON:
            # Handle left key press
            #self.camera.turn("LEFT")
            self.inputs_dict["SEMICOLON"] = keypress
        elif event.key == pygame.K_QUOTE:
            # Handle right key press
            #self.camera.turn("RIGHT")
            self.inputs_dict["QUOTE"] = keypress
        elif event.key == pygame.K_SPACE:
            if keypress:
                self.camera.toggle_orbit()
        elif event.key == pygame.K_s:
            if keypress:
                self.world.toggle_sunrising()
        elif event.key == pygame.K_RIGHTBRACKET:
            #self.camera.tilt("DOWN")
            self.inputs_dict["RIGHTBRACKET"] = keypress
        elif event.key == pygame.K_LEFTBRACKET:
            #self.camera.tilt("UP")
            self.inputs_dict["LEFTBRACKET"] = keypress

        elif event.key == pygame.K_a:

            self.inputs_dict["a"] = keypress

            
                
        elif event.key == pygame.K_d:
            self.inputs_dict["d"] = keypress

            
        elif event.key == pygame.K_1:
            if keypress:
                self.robot.current_joint_controlled = 0
                self.robot.updated = True
        elif event.key == pygame.K_2:
            if keypress:
                self.robot.current_joint_controlled = 1
                self.robot.updated = True
        elif event.key == pygame.K_3:
            if keypress:
                self.robot.current_joint_controlled = 2
                self.robot.updated = True
        elif event.key == pygame.K_4:
            if keypress:
                self.robot.current_joint_controlled = 3
                self.robot.updated = True
        elif event.key == pygame.K_5:
            if keypress:
                self.robot.current_joint_controlled = 4
                self.robot.updated = True
        elif event.key == pygame.K_6:
            if keypress:
                self.robot.current_joint_controlled = 5
                self.robot.updated = True
        elif event.key == pygame.K_7:
            if keypress:
                self.robot.current_joint_controlled = 6
                self.robot.updated = True

        elif event.key == pygame.K_z:
            if keypress:
                self.camera.fov+=1
        elif event.key == pygame.K_c:
            if keypress:
                self.camera.fov-=1
        elif event.key == pygame.K_w:
            if keypress:
                self.robot.wiggling = not self.robot.wiggling
        elif event.key == pygame.K_l:
            if keypress:
                self.robot.lock_pos = not self.robot.lock_pos
        elif event.key == pygame.K_EQUALS:
            if keypress:
                self.robot.add_arm()

        elif event.key == pygame.K_MINUS:
            if keypress:
                self.robot.remove_arm()

        elif event.key == pygame.K_r:
            if keypress:
                self.robot.load_from_json("robot2.json")

                print(f"Robot end effector pos: {self.robot.end_effector_pos}")
        elif event.key == pygame.K_t:
            if keypress:
                self.block.locked_to_arm = not self.block.locked_to_arm
                self.block.pos_to_end_effector = self.block.prisim.pos - self.robot.end_effector_pos

        elif event.key == pygame.K_F3:
            if keypress:
                self.display_developer_info = not self.display_developer_info


        


    def update(self):

        if self.current_view == "MENU":
            self.menu_button.is_hovered(pygame.mouse.get_pos())
            self.math_button.is_hovered(pygame.mouse.get_pos())

        
        self.camera.update(self.inputs_dict)

        self.robot.update(self.inputs_dict)

        self.block.update(self.robot.end_effector_pos,False)

        if self.robot.updated:
            self.robot.generate_arm_mesh()
            self.world.triangles = []
            self.populate_world()
            self.robot.updated = False


            # for i in range(self.robot.L):
            #     arm_control_gui = ArmControlGui(index=i)

        


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
    
    
    

    def convert_3d_to_2d_coords(self, camera_details: CameraDetails,point):

        P_rel_camera = point - camera_details.camera_pos

        P_rel_camera_height = np.dot(P_rel_camera, camera_details.camera_up)

        P_rel_camera_lateral = np.dot(P_rel_camera, camera_details.camera_horizontal)

        P_rel_camera_depth = np.dot(P_rel_camera, camera_details.camera_normal)


        P_prime_height = P_rel_camera_height * camera_details.view_dist / P_rel_camera_depth
        P_prime_lateral = P_rel_camera_lateral * camera_details.view_dist / P_rel_camera_depth


        camera_viewport_width =  2*camera_details.view_dist / math.tan(math.radians(self.camera.fov)) 
        camera_viewport_height = camera_viewport_width * (720.0 / 1080.0)

        camera_x = P_prime_lateral * camera_details.view_width / camera_viewport_width 
        camera_y = P_prime_height * camera_details.view_height / camera_viewport_height 

        in_viewport = True

        if abs(camera_x) > camera_details.view_width or abs(camera_y) > camera_details.view_height:
            in_viewport = False


        return np.array([camera_x, camera_y]), in_viewport
    
    
    def render_stats(self):
        text_surface = self.my_font.render(f'Camera pos: {self.camera.pos}', False, (0,0,0))
        
        self.screen.blit(text_surface,(0,0))
        text_surface = self.my_font.render(f'Camera tilt: {round(self.camera.pitch,2)}', False, (0,0,0))
        
        self.screen.blit(text_surface,(0,50))
        text_surface = self.my_font.render(f'Camera rotation: {round(self.camera.direction,2)}', False, (0,0,0))
        
        self.screen.blit(text_surface,(0,100))
        text_surface = self.my_font.render(f'Frame rate {round(self.fps,2)}', False, (0,0,0))
        
        self.screen.blit(text_surface,(0,150))

    def cam_coords_to_pygame(self,point):
        flip_y = np.array([point[0], -point[1]])
        move_to_center = flip_y + np.array([constants.WIDTH/2.0, constants.HEIGHT/2.0])

        return move_to_center
    
    def signed_distance_to_plane(self, point, plane_normal, d):
        return np.dot(plane_normal,np.array(point)) - d
    
    def clip_instance_against_plane(self, instance: Triangle, plane):

        d = self.signed_distance_to_plane(instance.get_center(),plane[0],plane[1])

        if d > 0:
            return True
        return False


    def clip_scene(self):


        # planes

        camera_facing = helpers.normalize(self.camera.facing)

        #print(self.camera.pos + 3 * camera_facing)

        near_plane = (camera_facing,helpers.signed_dist_to_plane(np.array([0,0,0]),camera_facing,(self.camera.pos + 3 * camera_facing)))
        far_plane = (-camera_facing,helpers.signed_dist_to_plane(np.array([0,0,0]),-camera_facing,(self.camera.pos + 90 * camera_facing)))

        #print(near_plane[1])
        clipped_triangles = []

        for triangle in self.world.triangles:

            # check if triangle needs to be clipped or not
            # start with simply clipping against the planes

            near = self.clip_instance_against_plane(triangle, near_plane)
            far = self.clip_instance_against_plane(triangle, far_plane)
            
            #print(far)
            if near and far:
                clipped_triangles.append(triangle)

        return clipped_triangles


    def render(self):

        if self.current_view == "MENU":
            self.menu_background.render(self.screen)

            self.menu_button.render(self.screen,self.my_font)
            self.math_button.render(self.screen,self.my_font)


            pass
        
        elif self.current_view == "GAME":
            
            
            camera_details = CameraDetails()
            camera_details.camera_normal = self.camera.facing
            camera_details.camera_up = self.camera.up
            camera_details.camera_horizontal = self.camera.get_perpendicular_axis()
            camera_details.camera_pos = self.camera.pos

            # need to process the triangles in the world and figure out
            # where to draw them relative to the camera

            self.screen.fill((255,255,255))

            self.screen.blit(self.game_background,(0,0))

            test_point = np.array([0,0,0])

            test_point_in_camera, in_camera = self.convert_3d_to_2d_coords(camera_details, test_point)

            test_point_in_camera = self.cam_coords_to_pygame(test_point_in_camera)

            pygame.draw.circle(self.screen,(255,155,0),(test_point_in_camera[0],test_point_in_camera[1]),5)
            
            clipped_triangles = self.clip_scene()
            

            for triangle in clipped_triangles:
                triangle.update_dist_to_camera(self.camera.pos)

            # sort triangles by distance to camera
            depth_buffer_triangles = sorted(clipped_triangles, key=lambda x: x.dist_to_camera, reverse=True)

            for triangle in depth_buffer_triangles:


                # check if i am looking at the triangle (if it is in our field of view)
                # do this by getting the line from the camera to each of the points

                in_camera = False

                new_points = []

                for point in triangle.points:


                    camera_to_point = point - self.camera.pos 
                    if np.dot(self.camera.facing, camera_to_point) > 0:
                        in_camera = True

                    new_point, point_in_camera = self.convert_3d_to_2d_coords(camera_details, point)

                    new_point = self.cam_coords_to_pygame(new_point)


                    

                    new_points.append(new_point)

                # get color of triangle
                lighted_color = triangle.get_lighted_color(self.world.sun_direction,self.camera.pos)

                if in_camera:
                
                    pygame.draw.polygon(self.screen,lighted_color,new_points)

                

                

                #pygame.draw.circle(self.screen,(0,255,0),(new_center[0],new_center[1]),1)


            pygame.draw.rect(self.screen,(0,0,0),pygame.Rect(constants.WIDTH/2-1,constants.HEIGHT/2-10,2,20))
            pygame.draw.rect(self.screen,(0,0,0),pygame.Rect(constants.WIDTH/2-10,constants.HEIGHT/2-1,20,2))

            if self.display_developer_info:
                self.render_stats()

            self.robot.render_control_bar(self.screen,self.my_font, [self.plus_icon,self.minus_icon])

            # # draw robot select bar
            # box_width = 40
            # pos = [constants.WIDTH / 2.0 - len(self.robot.L) / 2.0 * box_width,0]


            # selected_color = [13,41,63]
            # not_selected_color = [1,22,39]

            # selected_text = [162,191,252]
            # not_selected_text = [137,164,187]

            # for i in range(len(self.robot.L)):



            #     if i == self.robot.current_joint_controlled:
            #         box_color = selected_color
            #         text_color = selected_text
            #     else:
            #         box_color = not_selected_color
            #         text_color = not_selected_text


                

            #     pygame.draw.rect(self.screen,box_color,pygame.Rect(pos[0] + 40 * i,pos[1],40,40))
            #     pygame.draw.rect(self.screen,not_selected_color,pygame.Rect(pos[0]+40 * i,40,20,20))
            #     self.screen.blit(self.plus_icon,(pos[0]+40*i,40))
            #     pygame.draw.rect(self.screen,not_selected_color,pygame.Rect(pos[0]+40 * i + 20,40,20,20))
            #     self.screen.blit(self.minus_icon,(pos[0]+40*i + 20,40))
                
            #     text_surface = self.my_font.render(f'{i+1}', False, text_color)
            #     text_rect = text_surface.get_rect(center=(pos[0] + 40 * i + box_width/2.0, pos[1] + box_width / 2.0))
            #     self.screen.blit(text_surface,text_rect)



                



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
            self.fps = 1.0 / (time.time() - start_time)
            #pygame.display.set_caption(f"FPS: {round(fps,2)}")

