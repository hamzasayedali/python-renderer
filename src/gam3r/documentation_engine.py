import pygame
import time

WIDTH = 1080
HEIGHT = 720

class Engine:
    def __init__(self):
        pass

    def startup(self):
        try:
            #Initalize Pygame
            pygame.init()
            #Create Window with custom title
            pygame.display.set_caption("Pygame 3D Engine")
            self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
            pygame.display.flip()

        
        except Exception as e:
            print(e)
            exit()

    def inputs(self,event):
        pass
    def update(self):
        pass
    def render(self):
        self.screen.fill([250,250,250])

        pygame.draw.circle(self.screen,[255,0,0],[WIDTH/2,HEIGHT/2],5)

        pygame.draw.polygon(self.screen,[0,255,200],[[400,100],[350,200],[450,200]])

        pygame.display.update()
        pass

    def run(self):

        self.startup()

        while True:
            # get the user's inputs
            try:
                #input handling
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        pygame.quit()

                        break
                    
                    # Check for key presses
                    elif event.type == pygame.KEYDOWN or event.type == pygame.KEYUP or event.type == pygame.MOUSEBUTTONDOWN:
                        self.inputs(event)

                # update any game objects based on the user's inputs/game clock
                self.update()
                # draw the game objects to the screen
                self.render()

            except KeyboardInterrupt:
                
                pygame.quit()
                
                break

            #small delay to avoid maxxing out the CPU usage
            time.sleep(0.01)

        
engine = Engine()

engine.run()