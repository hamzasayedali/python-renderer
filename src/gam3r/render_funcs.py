import numpy as np

class CameraDetails:
    def __init__(self):
        self.camera_pos = np.array([3,0,1])
        self.camera_normal = np.array([-1,0,0])
        self.camera_up = np.array([0,0,1])
        self.camera_horizontal = np.cross(self.camera_normal, self.camera_up)
        self.view_width = 1080.0
        self.view_height = 720.0
        self.view_dist = 3.0


test_pos = np.array([0,0,0])

def convert_3d_to_2d_coords(camera_details: CameraDetails,point):

    P_rel_camera = point - camera_details.camera_pos

    print(P_rel_camera)

    P_rel_camera_height = np.dot(P_rel_camera, camera_details.camera_up)

    P_rel_camera_lateral = np.dot(P_rel_camera, camera_details.camera_horizontal)

    P_rel_camera_depth = np.dot(P_rel_camera, camera_details.camera_normal)

    P_prime_height = P_rel_camera_height * camera_details.view_dist / P_rel_camera_depth
    P_prime_lateral = P_rel_camera_lateral * camera_details.view_dist / P_rel_camera_depth


    camera_viewport_width =  3.0 
    camera_viewport_height = camera_viewport_width * (720.0 / 1080.0)


    camera_x = P_prime_lateral * camera_details.view_width / camera_viewport_width 
    camera_y = P_prime_height * camera_details.view_height / camera_viewport_height 


    return np.array([camera_x, camera_y])



camera_details = CameraDetails()

print(convert_3d_to_2d_coords(camera_details, test_pos))