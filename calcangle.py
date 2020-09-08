import numpy as np
import math as math

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)

    print(v1_u)
    print(v2_u)
    print("---------------")
    print(np.dot(v1_u,v2_u))
    arcangle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    return arcangle * 180 /math.pi

""" 'v1 = np.array([1,0,0]) 
'v2 = np.array([-1,0,0])  """

n0 = np.array([0.569275,1,-3.06802])

pv0 = np.array([0.546311,0.161109,-3.00472]) 
pv1 = np.array([0.569275,-0.38531,-3.06802]) 

v1 = n0 - pv1
v2 = pv0 - pv1

v3 = np.array([-0.0151746,0.572051,0.0331497,0]) 
v4 = np.array([0,1.39739,0,0]) 
