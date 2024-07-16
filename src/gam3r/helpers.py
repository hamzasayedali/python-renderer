import numpy as np
import math

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

    print(plane_normal * point_on_plane)

    dist = (np.sum(plane_normal * point) + D) / math.sqrt(np.sum(plane_normal * plane_normal))

    return dist



plane_normal = np.array([1,0,0])

point = np.array([0,110,10])
point_on_plane = np.array([3,0,0])

print(signed_dist_to_plane(point, plane_normal, point_on_plane))