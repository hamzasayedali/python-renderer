import pygame
import time
import numpy as np
import math
import helpers
import json
import random

import models as models

from game_objs import Camera, World, Triangle, Prisim, Robot, Skybox, PointGridPlane, MenuButton, MenuBackground, hallway, Floor, Boat, ArmControlGui, Block, TextBox, Octahedron, Tetrahedron, Icosahedron, ObjFile, Player, Coin

from render_funcs import CameraDetails
import constants

class Engine:

    def __init__(self, name):
        self.name = name
        self.screen = None

        self.current_view = "MENU"

        self.current_level = 0
        self.current_level_name = "Sample Level"

        self.num_levels = 4

        # used to measure how long function calls take
        self.profile = False
        self.profiler_functions = []
        self.profiler_job_count = []
        self.profiler_times = []

        

        # store all the buttons that are on the screen currently, 
        # can loop through when the mouse is clicked to check for collisions and trigger the required functions.
        self.buttons = [
            MenuButton("Next Level", (constants.WIDTH/2.0 + 300,constants.HEIGHT-40),40,self.level_increment),
            MenuButton("Prev Level", (constants.WIDTH/2.0 - 300,constants.HEIGHT-40),40,self.level_decrement)
        ]

        self.level_1_buttons = [
            MenuButton("Cube", (20,20),40,on_click=self.select_cube,align="left"),
            MenuButton("Tetrahedron", (20,60),40,on_click=self.select_tetrahedron,align="left"),
            MenuButton("Octahedron", (20,100),40,on_click=self.select_octahedron,align="left"),
            MenuButton("Icosahedron", (20,140),40,on_click=self.select_icosahedron,align="left"),
            MenuButton("Person", (20,180),40,on_click=self.select_person,align="left"),
            MenuButton("Teapot", (20,220),40,on_click=self.select_teapot,align="left"),
            MenuButton("Gourd", (20,260),40,on_click=self.select_gourd,align="left"),
            MenuButton("Cow", (20,300),40,on_click=self.select_cow,align="left"),
            MenuButton("Dodecahedron", (20,340),40,on_click=self.select_dodecahedron,align="left"),
            
        ]

        self.world = World()
        self.camera = Camera()

        self.load_level_info(self.current_level)

        self.robot = Robot()
        self.block = Block()
        self.current_robot_joint = 0
        self.camera_to_player = np.array([15,0,-14])
        self.player = Player(self.camera.pos + self.camera_to_player)

        self.display_developer_info = False

        self.fps = 0

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

        self.return_to_menu_button = MenuButton("<-", (constants.WIDTH - 25,25))
        self.menu_background = MenuBackground()



        self.game_background = pygame.image.load('./assets/game_background.jpg')
        self.game_background = pygame.transform.scale(self.game_background, (constants.WIDTH, constants.HEIGHT))

        self.plus_icon = pygame.image.load('./assets/plus.png')
        self.minus_icon = pygame.image.load('./assets/minus.png')


    def load_level_info(self, level):
        f = open(f"./src/gam3r/levels/{level}.json", "r")
        x = f.read()
        y = json.loads(x)

        camera_position = y["camera_position"]
        camera_direction = y["camera_direction"]
        camera_tilt = y["camera_tilt"]
        camera_FOV = y["camera_FOV"]

        level_name = y["level_name"]

        self.current_level_name = level_name

        

        self.camera = Camera(pos=camera_position, direction=camera_direction,pitch=camera_tilt,fov=camera_FOV)

        floor = Floor(y["floor"]["origin"],y["floor"]["tile_count"],y["floor"]["color"])


        print(y["floor"]["origin"])
        self.world = World()

        self.world.entities.append(floor)


        if level == 0:

            triangle = Triangle(
            [[0,2,0],[0,-2,0],[0,0,2]],[150,150,200]
            )
            self.world.entities.append(triangle)

            triangle2 = Triangle(
            [[-2,4,0],[-2,2,0],[-2,3,2]],[250,150,200]
            )
            self.world.entities.append(triangle2)

            triangle3 = Triangle(
            [[-2,-4,0],[-2,-2,0],[-2,-3,2]],[50,150,100]
            )
            self.world.entities.append(triangle3)

            triangle4 = Triangle(
            [[-6,4,0],[-6,2,0],[-6,3,2]],[67,214,30]
            )
            self.world.entities.append(triangle4)

            triangle5 = Triangle(
            [[-6,-4,0],[-6,-2,0],[-6,-3,2]],[90,75,118]
            )
            self.world.entities.append(triangle5)

        if level == 1:

            cube = Prisim([0,0,3],[2,2,2],[0,0,1],[1,0,0],[23,76,130])

            self.world.entities.append(cube)

        if level == 2:

            self.world.entities.append(self.robot)

        if level == 3:
            self.world.score = 0
            self.world.sun_direction = np.array([-200,100,50])
            self.camera.velocity = np.array([0.3,0,0])

            shuttle = ObjFile([0,0,0],[1,1,1],up=[0,0,1], facing=[1,0,0], color=[250,100,100],filename="hallway.obj",scale=1,invert_mesh=True)
            self.world.entities.append(shuttle)
            self.player = Player(self.camera.pos + self.camera_to_player)
            self.world.entities.append(self.player)

            

            for i in range(200):
                shuttle = ObjFile([5+5*i,0,0],[1,1,1],up=[0,0,1], facing=[1,0,0], color=[random.randint(20,230),random.randint(20,230),random.randint(20,230)],filename="hallway.obj",scale=1,invert_mesh=True)
                self.world.entities.append(shuttle)

            for i in range(30):
                self.world.coins.append(Coin([33+i*10,random.randint(-9,-1),3]))

            


        

    def level_increment(self):
        print(f"Current level is {self.current_level}")
        if self.current_level < self.num_levels - 1:
            self.current_level += 1
            self.update_level()
            print(f"Next level is {self.current_level}")
        else:
            print("This is the last level")

    def level_decrement(self):
        print(f"Current level is {self.current_level}")
        if self.current_level > 0:
            self.current_level -= 1
            self.update_level()
            print(f"Next level is {self.current_level}")
        else:
            print("This is the first level")

    def update_level(self):
        self.load_level_info(self.current_level)
        self.populate_world()


    def select_cube(self):

        new_entities = []
        for obj in self.world.entities:
            if isinstance(obj, Floor):
                new_entities.append(obj)



        for i in range(10):

            cube = Prisim([random.randint(-20,20),random.randint(-20,20),random.randint(1,20)],[2,2,2],[0,0,1],[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)])
            new_entities.append(cube)

        self.world.entities = new_entities
        self.world.triangles = []
        self.populate_world()
        print("Cube!")
    def select_octahedron(self):
        new_entities = []
        for obj in self.world.entities:
            if isinstance(obj, Floor):
                new_entities.append(obj)

        for i in range(10):
            octahedron = Octahedron([random.randint(-20,20),random.randint(-20,20),random.randint(4,24)],[2,2,4],helpers.normalize(np.array([0,random.randint(-10,10)/10.0,random.randint(-10,10)/10.0])),[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)])
            new_entities.append(octahedron)
        self.world.entities = new_entities
        self.world.triangles = []
        self.populate_world()
        
    def select_tetrahedron(self):
        new_entities = []
        for obj in self.world.entities:
            if isinstance(obj, Floor):
                new_entities.append(obj)
        tetrahedron = Tetrahedron([0,0,5],[6,2,4],helpers.normalize(np.array([0,0,1])),[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)])
        new_entities.append(tetrahedron)
        self.world.entities = new_entities
        self.world.triangles = []
        self.populate_world()
        print("Tetrahedron!")
    def select_dodecahedron(self):
        print("Dodecahedron!")
    def select_icosahedron(self):
        new_entities = []
        for obj in self.world.entities:
            if isinstance(obj, Floor):
                new_entities.append(obj)
        icosahedron = Icosahedron([0,0,5],[6,2,4],helpers.normalize(np.array([0,0,1])),[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)])
        new_entities.append(icosahedron)
        self.world.entities = new_entities
        self.world.triangles = []
        self.populate_world()
        print("Tetrahedron!")
        print("Icosahedron!")
    def select_person(self):
        new_entities = []
        for obj in self.world.entities:
            if isinstance(obj, Floor):
                new_entities.append(obj)
        humanoid = ObjFile([0,0,5],[6,2,4],helpers.normalize(np.array([0,0,1])),[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)],"humanoid_tri.obj",scale=0.5,invert_mesh=True)
        new_entities.append(humanoid)
        self.world.entities = new_entities
        self.world.triangles = []
        self.populate_world()
    def select_teapot(self):
        new_entities = []
        for obj in self.world.entities:
            if isinstance(obj, Floor):
                new_entities.append(obj)
        humanoid = ObjFile([0,0,5],[6,2,4],helpers.normalize(np.array([0,0,1])),[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)],"teapot.obj",scale=0.1,invert_mesh=False)
        new_entities.append(humanoid)
        self.world.entities = new_entities
        self.world.triangles = []
        self.populate_world()
    def select_gourd(self):
        new_entities = []
        for obj in self.world.entities:
            if isinstance(obj, Floor):
                new_entities.append(obj)
        gourd = ObjFile([0,0,5],[6,2,4],helpers.normalize(np.array([0,0,1])),[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)],"gourd.obj",scale=3,invert_mesh=True)
        new_entities.append(gourd)
        self.world.entities = new_entities
        self.world.triangles = []
        self.populate_world()
    def select_cow(self):
        new_entities = []
        for obj in self.world.entities:
            if isinstance(obj, Floor):
                new_entities.append(obj)
        cow = ObjFile([0,0,5],[6,2,4],helpers.normalize(np.array([0,0,1])),[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)],"cow.obj",scale=3,invert_mesh=True)
        new_entities.append(cow)
        self.world.entities = new_entities
        self.world.triangles = []
        self.populate_world()
    def select_dodecahedron(self):
        new_entities = []
        for obj in self.world.entities:
            if isinstance(obj, Floor):
                new_entities.append(obj)
        dodecahedron = ObjFile([0,0,5],[6,2,4],helpers.normalize(np.array([0,0,1])),[1,0,0],[random.randint(20,230),random.randint(20,230),random.randint(20,230)],"dodecahedron.obj",scale=3,invert_mesh=True)
        new_entities.append(dodecahedron)
        self.world.entities = new_entities
        self.world.triangles = []
        self.populate_world()



    def populate_world(self):
        self.world.load_entities()

        print(len(self.world.triangles))

    def populate_world_2(self):

        
        
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

        game_floor = Floor([-16,-16,-1],8)

        #for triangle in hall:
        #    self.world.add_triangle(triangle)

        for triangle in game_floor.triangles:
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
            self.my_font = pygame.font.SysFont('Arial', 30)
        
        except Exception as e:
            print(e)
            exit()

        # startup functions.
        self.populate_world()

        print(len(self.world.triangles))


    def inputs(self, event):

        keypress = False

        

        if event.type == pygame.MOUSEBUTTONDOWN:
            
            mouse_pos = pygame.mouse.get_pos()
            self.robot.update_mouse_clicks(mouse_pos)

            if self.current_view == "MENU":
                start_is_clicked = self.menu_button.update_mouse_clicks(mouse_pos)
                if start_is_clicked:
                    self.current_view = "GAME"
                self.math_button.update_mouse_clicks(mouse_pos)

            elif self.current_view == "GAME":
                exit_is_clicked = self.return_to_menu_button.update_mouse_clicks(mouse_pos)
                if exit_is_clicked:
                    self.current_view = "MENU"


            for button in self.buttons:
                button.update_mouse_clicks(mouse_pos)


            if self.current_level == 1:
                for button in self.level_1_buttons:
                    button.update_mouse_clicks(mouse_pos)
                    
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
        
        elif event.key == pygame.K_p:
            if keypress:
                self.profile = not self.profile
        elif event.key == pygame.K_g:
            if keypress:
                self.camera.toggle_path_follow()


    def update(self):




        
        self.menu_button.hovered = False
        self.math_button.hovered = False
        self.return_to_menu_button.hovered = False
        if self.current_view == "MENU":
            self.menu_button.is_hovered(pygame.mouse.get_pos())
            self.math_button.is_hovered(pygame.mouse.get_pos())

        if self.current_view == "GAME":
            self.return_to_menu_button.is_hovered(pygame.mouse.get_pos())
            

        for button in self.buttons:
            button.is_hovered(pygame.mouse.get_pos())

        if self.current_level == 1:
            for button in self.level_1_buttons:
                button.is_hovered(pygame.mouse.get_pos())
        
        self.camera.update(self.inputs_dict)








        if self.current_level == 2:
            self.robot.update(self.inputs_dict)

            #self.block.update(self.robot.end_effector_pos,False)

            if self.robot.updated:
                self.robot.generate_arm_mesh()

                new_entities = []
                for obj in self.world.entities:
                    if not isinstance(obj,Robot):
                        new_entities.append(obj)
                new_entities.append(self.robot)

                self.world.entities = new_entities

                
                self.world.triangles = []
                self.populate_world()
                self.robot.updated = False


                #for i in range(len(self.robot.L)):
                #    arm_control_gui = ArmControlGui(index=i)

        if self.current_level == 3:
            self.player = Player(self.camera.pos+self.camera_to_player)
            new_entities = []
            for obj in self.world.entities:
                if not isinstance(obj,Player):
                    new_entities.append(obj)
            new_entities.append(self.player)

            self.world.entities = new_entities

            
            self.world.triangles = []
            self.populate_world()

            if self.camera.pos[1] < -9.5:
                self.camera.pos[1] = -9.5

            if self.camera.pos[1] > -0.5:
                self.camera.pos[1] = -0.5


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

        if P_rel_camera_depth == 0:
            P_rel_camera_depth = 0.01
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

        camera_facing = self.camera.facing

        #print(self.camera.pos + 3 * camera_facing)

        near_plane = (camera_facing,helpers.signed_dist_to_plane(np.array([0,0,0]),camera_facing,(self.camera.pos + 6 * camera_facing)))
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
    
    def remove_not_facing_triangles(self, triangles):
        result = []
        camera_facing = helpers.normalize(self.camera.facing)

        for triangle in triangles:
            #if np.dot(camera_facing, triangle.get_normal()) > 0:
            if np.dot(triangle.get_normal(),triangle.get_center()-self.camera.pos) > 0:
                result.append(triangle)

        return result




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

            p_clock = time.time()

            clipped_triangles = self.clip_scene()

            if self.profile:
                self.profiler_functions.append("clipped_triangles")
                self.profiler_job_count.append(len(self.world.triangles))
                new_clock = time.time()
                self.profiler_times.append(new_clock - p_clock)

            
            p_clock = time.time()

            facing_triangles = self.remove_not_facing_triangles(clipped_triangles)
            
            if self.profile:
                self.profiler_functions.append("facing_triangles")
                self.profiler_job_count.append(len(clipped_triangles))
                new_clock = time.time()
                self.profiler_times.append(new_clock - p_clock)

            
            p_clock = time.time()

            for triangle in facing_triangles:
                triangle.update_dist_to_camera(self.camera.pos)
                
            if self.profile:
                self.profiler_functions.append("update_dist_to_camera")
                self.profiler_job_count.append(len(facing_triangles))
                new_clock = time.time()
                self.profiler_times.append(new_clock - p_clock)


            p_clock = time.time()
            # sort triangles by distance to camera
            depth_buffer_triangles = sorted(facing_triangles, key=lambda x: x.dist_to_camera, reverse=True)
            
            if self.profile:
                self.profiler_functions.append("depth_buffer_triangles")
                self.profiler_job_count.append(len(facing_triangles))
                new_clock = time.time()
                self.profiler_times.append(new_clock - p_clock)

            p_clock = time.time()
            for triangle in depth_buffer_triangles:

                # check if i am looking at the triangle (if it is in our field of view)
                # do this by getting the line from the camera to each of the points

                in_camera = False

                new_points = []

                # check if the triangle is facing me, e.g. the normal of the triangle
                # dotted with the normal of the camera to the triangle
                # is positive

                for point in triangle.points:


                    camera_to_point = point - self.camera.pos 
                    if np.dot(self.camera.facing, camera_to_point) > 0:
                        in_camera = True

                    q_clock = time.time()
                    new_point, point_in_camera = self.convert_3d_to_2d_coords(camera_details, point)

                    if self.profile:
                        self.profiler_functions.append("convert_3d_to_2d_coords")
                        self.profiler_job_count.append(1)
                        new_clock = time.time()
                        self.profiler_times.append(new_clock - q_clock)

                    new_point = self.cam_coords_to_pygame(new_point)


                    

                    new_points.append(new_point)

                # get color of triangle
                q_clock = time.time()

                
                lighted_color = triangle.get_lighted_color(self.world.sun_direction,self.camera.pos)
                    
                
                if self.profile:
                    self.profiler_functions.append("get_lighted_color")
                    self.profiler_job_count.append(1)
                    new_clock = time.time()
                    self.profiler_times.append(new_clock - q_clock)

                if in_camera:
                    q_clock = time.time()
                    pygame.draw.polygon(self.screen,lighted_color,new_points)
                    if self.profile:
                        self.profiler_functions.append("pygame.draw.polygon")
                        self.profiler_job_count.append(1)
                        new_clock = time.time()
                        self.profiler_times.append(new_clock - q_clock)

            if self.profile:
                self.profiler_functions.append("draw_loop")
                self.profiler_job_count.append(len(depth_buffer_triangles))
                new_clock = time.time()
                self.profiler_times.append(new_clock - p_clock)



            pygame.draw.rect(self.screen,(0,0,0),pygame.Rect(constants.WIDTH/2-1,constants.HEIGHT/2-10,2,20))
            pygame.draw.rect(self.screen,(0,0,0),pygame.Rect(constants.WIDTH/2-10,constants.HEIGHT/2-1,20,2))

            

            if self.current_level == 2:

                self.robot.render_control_bar(self.screen,self.my_font, [self.plus_icon,self.minus_icon])

            if self.display_developer_info:
                self.render_stats()

            self.return_to_menu_button.render(self.screen,self.my_font)

            for button in self.buttons:
                button.render(self.screen,self.my_font)

            if self.current_level == 1:
                for button in self.level_1_buttons:
                    button.render(self.screen,self.my_font)

            if self.current_level == 3:
                TextBox(text=f"Score: {self.world.score}", pos=(100,40)).render(self.screen,self.my_font)



            TextBox(text=f"Level {self.current_level+1}: {self.current_level_name}", pos=(constants.WIDTH/2.0,constants.HEIGHT-40)).render(self.screen,self.my_font)
            



                



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
                        print(self.profiler_functions)
                        print(self.profiler_job_count)
                        print(self.profiler_times)

                        helpers.profiling(self.profiler_functions,self.profiler_job_count,self.profiler_times)
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

            #time.sleep(0.005)
            clock.tick(600)
            self.fps = 1.0 / (time.time() - start_time)
            #pygame.display.set_caption(f"FPS: {round(fps,2)}")

