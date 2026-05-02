import numpy as np
import math
import xml.etree.ElementTree as ET
import pickle
import networkx as nx


def rpyxyz2H(rpy: np.ndarray, xyz:np.ndarray)->np.ndarray:
    """
    Computes the homogeneous transformation matrix given rpy and xyz
    
    Args: 
        - rpy: 3x1 roll-pitch-yaw angles
        - xyz: 3x1 xyz position
        
    Returns:
        - H: 4x4 homogeneous transformation matrix
    """
    Ht = [[1, 0, 0, xyz[0]],
          [0, 1, 0, xyz[1]],
          [0, 0, 1, xyz[2]],
          [0, 0, 0, 1]]

    Hx = [[1, 0, 0, 0],
          [0, math.cos(rpy[0]), -math.sin(rpy[0]), 0],
          [0, math.sin(rpy[0]), math.cos(rpy[0]), 0],
          [0, 0, 0, 1]]

    Hy = [[math.cos(rpy[1]), 0, math.sin(rpy[1]), 0],
          [0, 1, 0, 0],
          [-math.sin(rpy[1]), 0, math.cos(rpy[1]), 0],
          [0, 0, 0, 1]]

    Hz = [[math.cos(rpy[2]), -math.sin(rpy[2]), 0, 0],
          [math.sin(rpy[2]), math.cos(rpy[2]), 0, 0],
          [0, 0, 1, 0],
          [0, 0, 0, 1]]

    Ht = np.matmul(np.matmul(np.matmul(Ht, Hz), Hy), Hx)

    return Ht


def R2axisang(R: np.ndarray)->(np.ndarray, float):
    """
    Computes the axis and angle of a rotation matrix
    
    Args:
        - R: 3x3 rotation matrix
    
    Returns:
        - axis: 3x1 axis of rotation
        - ang: angle of rotation
    """
    ang = math.acos((R[0, 0] + R[1, 1] + R[2, 2] - 1)/2)
    Z = np.linalg.norm(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if Z == 0:
        return [1, 0, 0], 0.
    x = (R[2, 1] - R[1, 2])/Z
    y = (R[0, 2] - R[2, 0])/Z
    z = (R[1, 0] - R[0, 1])/Z
    return [x, y, z], ang

def MatrixExp(axis: np.ndarray, theta: float)->np.ndarray:
    """
    Computes the matrix exponential of a rotation matrix
    """
    so3_axis = so3(axis)
    R = np.eye(3) + np.sin(theta)*so3_axis + \
        (1 - np.cos(theta))*np.matmul(so3_axis, so3_axis)
    last = np.zeros((1, 4))
    last[0, 3] = 1
    H_r = np.vstack((np.hstack((R, np.zeros((3, 1)))), last))
    return H_r

def so3(axis: np.ndarray)->np.ndarray:
    """
    Returns the skew symmetric matrix of a vector
    """
    so3_axis = np.asarray([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    return so3_axis


def FindNearest(prevPoints,newPoint):
    """
    Returns the index of the nearest point in the prevPoints
    """
    D=np.array([np.linalg.norm(np.array(point)-np.array(newPoint)) for point in prevPoints])
    return D.argmin()




def BlockDesc2Points(H, Dim):
	center = H[0:3,3]
	axes=[ H[0:3,0],H[0:3,1],H[0:3,2]]	

	corners=[
		center,
		center+(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center+(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.),
		center+(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center+(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)+(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)+(axes[2]*Dim[2]/2.),
		center-(axes[0]*Dim[0]/2.)-(axes[1]*Dim[1]/2.)-(axes[2]*Dim[2]/2.)
		]	

	return corners, axes



def CheckPointOverlap(pointsA,pointsB,axis):

	#Project points
	projPointsA=np.matmul(axis, np.transpose(pointsA))
	projPointsB=np.matmul(axis, np.transpose(pointsB))
	
	#Check overlap
	maxA= np.max(projPointsA)
	minA= np.min(projPointsA)
	maxB= np.max(projPointsB)
	minB= np.min(projPointsB)
	
	if maxA<=maxB and maxA>=minB:
		return True

	if minA<=maxB and minA>=minB:
		return True

	if maxB<=maxA and maxB>=minA:
		return True

	if minB<=maxA and minB>=minA:
		return True

	return False



def CheckBoxBoxCollision(pointsA,axesA,pointsB,axesB):
	
	#Sphere check - first point is box center
	if np.linalg.norm(pointsA[0]-pointsB[0])> (np.linalg.norm(pointsA[0]-pointsA[1])+np.linalg.norm(pointsB[0]-pointsB[1])):
		return False
	

	#Surface normal check
	for i in range(3):
		if not CheckPointOverlap(pointsA,pointsB, axesA[i]):
			return False

	for j in range(3):
		if not CheckPointOverlap(pointsA,pointsB, axesB[j]):
			return False


	#Edge edge check
	for i in range(3):
		for j in range(3):
			if not CheckPointOverlap(pointsA,pointsB, np.cross(axesA[i],axesB[j])):
				return False

	return True
	
def axis_angle_between(v1, v2): #ChatGPT generated
    """
    Compute the axis and angle (in radians) required to rotate v1 to align with v2.
    Both v1 and v2 should be 3D vectors.
    Returns:
        axis: unit vector representing the rotation axis
        angle: rotation angle in radians
    """
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    # Dot and cross products
    dot = np.dot(v1, v2)
    cross = np.cross(v1, v2)

    # Clamp dot to avoid numerical errors outside [-1, 1]
    dot = np.clip(dot, -1.0, 1.0)
    angle = np.arccos(dot)

    if np.isclose(angle, 0.0):
        # Vectors are already aligned
        return np.array([1, 0, 0]), 0.0
    elif np.isclose(angle, np.pi):
        # Vectors are opposite -> choose orthogonal axis
        orth = np.array([1, 0, 0])
        if np.allclose(v1, orth) or np.allclose(v1, -orth):
            orth = np.array([0, 1, 0])
        axis = np.cross(v1, orth)
        axis /= np.linalg.norm(axis)
        return axis, np.pi
    else:
        axis = cross / np.linalg.norm(cross)
        return axis, angle

def interp_min_jerk(q_start, q_goal, t, T):
    tau = np.clip(t / max(T, 1e-6), 0.0, 1.0)
    s = 10*tau**3 - 15*tau**4 + 6*tau**5
    ds = (30*tau**2 - 60*tau**3 + 30*tau**4) / max(T, 1e-6)

    q_des = q_start + s * (q_goal - q_start)
    qd_des = ds * (q_goal - q_start)
    
    return q_des, qd_des

def add_free_block_to_model(tree, name, pos, density, size, rgba, free):
    worldbody = tree.getroot().find("worldbody")

    body = ET.SubElement(worldbody, "body", {"name": {name},"pos": f"{pos[0]} {pos[1]} {pos[2]}",})
    ET.SubElement(body, "geom", {"type": "box", "density": f"{density}", "size": f"{size[0]} {size[1]} {size[2]}","rgba": f"{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}",})
    if free is True: ET.SubElement(body, "freejoint")  #make it a freebody

    return

def AnalyzeGraph(prmVertices=None, prmEdges=None, prm_file="myPRM.p"):
    if prmVertices is None or prmEdges is None:
        with open(prm_file, 'rb') as f:
            prmVertices = pickle.load(f)
            prmEdges = pickle.load(f)

    G = nx.Graph()
    G.add_nodes_from(range(len(prmVertices)))
    for v, neighbors in enumerate(prmEdges):
        for u in neighbors:
            G.add_edge(v, u)
    return G.number_of_nodes(), G.number_of_edges(), nx.number_connected_components(G)