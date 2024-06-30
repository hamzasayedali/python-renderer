from engine import Triangle
import numpy as np

from helpers import angle_between_vectors_degrees

from engine import Camera

def test_triangle_lighting():
    triangle = Triangle([
            (0,0,-2),
            (-1,1,0),
            (-1,-1,0)])
    
    triangle6 = Triangle([
            (-1,-1,0),
            (1,-1,0),
            (0,0,2)])
    
    sun_direction = np.array([4,0,2])
    
    print(triangle.get_lighted_color(sun_direction))
    print(triangle6.get_lighted_color(sun_direction))

test_triangle_lighting()

def test_camera_orbit():
    camera = Camera(pos=[1,2,0])

    print(camera.orbit([0,0,0]))


def test_angles_helper():

    vector1 = [1,0,0]
    vector2 = [-1,2,0]
    norm = [0,0,1]

    print(angle_between_vectors_degrees(vector1,vector2,norm))

test_angles_helper()