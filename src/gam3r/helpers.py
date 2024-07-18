import numpy as np
import math
from numba import njit

def angle_between_vectors_degrees(v1, v2, norm=[0,0,1]):
    dot_prod = np.dot(v1, v2)

    det = np.dot(norm,np.cross(v1,v2))

    angle_rad = math.atan2(dot_prod,det)
    angle_deg = math.degrees(angle_rad) + 90
    
    return angle_deg

def normalize(vec):

    norm = np.linalg.norm(vec)

    return vec / norm
    
def signed_dist_to_plane(point, plane_normal, point_on_plane):
    D = np.sum(plane_normal * point_on_plane)

    #print(plane_normal * point_on_plane)

    dist = (np.sum(plane_normal * point) + D) / math.sqrt(np.sum(plane_normal * plane_normal))

    return dist

# @njit()
# def dot_3d(a=[0,0,0],b=[0,0,0]):
    
#     return np.array(a[0]*b[0] + a[1]*b[1] + a[2]*b[2])


# @njit()
# def jit_convert_3d_to_2d_coords(camera_pos, camera_up, camera_horizontal,camera_normal, view_dist, view_width, view_height,fov,point:np.float64):

#     P_rel_camera = point - camera_pos

#     P_rel_camera_height = dot_3d(P_rel_camera, camera_up)

#     P_rel_camera_lateral = dot_3d(P_rel_camera, camera_horizontal)

#     P_rel_camera_depth = dot_3d(P_rel_camera, camera_normal)


#     P_prime_height = P_rel_camera_height * view_dist / P_rel_camera_depth
#     P_prime_lateral = P_rel_camera_lateral * view_dist / P_rel_camera_depth


#     camera_viewport_width =  2*view_dist / math.tan(math.radians(fov)) 
#     camera_viewport_height = camera_viewport_width * (720.0 / 1080.0)

#     camera_x = P_prime_lateral * view_width / camera_viewport_width 
#     camera_y = P_prime_height * view_height / camera_viewport_height 

#     in_viewport = True

#     if abs(camera_x) > view_width or abs(camera_y) > view_height:
#         in_viewport = False


#     return np.array([camera_x, camera_y]), in_viewport



plane_normal = np.array([1,0,0])

point = np.array([0,110,10])
point_on_plane = np.array([3,0,0])

print(signed_dist_to_plane(point, plane_normal, point_on_plane))