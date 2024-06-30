import numpy as np
import math

def angle_between_vectors_degrees(v1, v2, norm=[0,0,1]):
    dot_prod = np.dot(v1, v2)

    det = np.dot(norm,np.cross(v1,v2))

    angle_rad = math.atan2(dot_prod,det)
    angle_deg = math.degrees(angle_rad) - 90
    
    return angle_deg