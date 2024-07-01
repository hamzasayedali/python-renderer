from engine import Triangle
import numpy as np

from helpers import angle_between_vectors_degrees

from engine import Camera

from game_objs import Prisim

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
    
    print(triangle.get_lighted_color(sun_direction, [10,0,0]))
    print(triangle6.get_lighted_color(sun_direction, [10,0,0]))

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


def test_camera_tilt():
    camera = Camera()

    print(f"Facing: {camera.facing} Up: {camera.up} Pitch: {camera.pitch} Direction: {camera.direction}")
    

    camera.tilt("UP")

    print(f"Facing: {camera.facing} Up: {camera.up} Pitch: {camera.pitch} Direction: {camera.direction}")

    camera.tilt("DOWN")

    print(f"Facing: {camera.facing} Up: {camera.up} Pitch: {camera.pitch} Direction: {camera.direction}")

    camera.tilt("UP")

    print(f"Facing: {camera.facing} Up: {camera.up} Pitch: {camera.pitch} Direction: {camera.direction}")
    

test_camera_tilt()

def test_prisim():
    prisim = Prisim([-6,3,0],[1,1,4])

test_prisim()