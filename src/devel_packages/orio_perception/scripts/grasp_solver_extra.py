import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

def compute_mask_com(mask):
    ys, xs = np.where(mask)

    if len(xs) == 0:
        return None

    return (float(np.median(xs)), float(np.median(ys)))


def optimize_grasp_pose(pcd, cup_radius, dist_between_cups, theta_allowed, min_points_for_seal):
    """
    Deterministic grasp calculator for planar objects.
    Extracts the largest plane, centers on it, and aligns with the longest axis.
    """
    if len(pcd.points) < 100:
        return None, "Not enough points in point cloud."

    # 1. Isolate the largest planar surface using RANSAC
    # distance_threshold is how thick the plane can be (in meters). 
    # 0.015m (1.5cm) is usually a good tolerance for depth camera noise on a flat box.
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.015,
                                             ransac_n=3,
                                             num_iterations=1000)
    
    if len(inliers) < min_points_for_seal * 2:
        return None, "No planar surface found large enough to grasp."

    # Create a sub-cloud of ONLY the flat points (strips away curves/sides)
    inlier_cloud = pcd.select_by_index(inliers)
    points = np.asarray(inlier_cloud.points)
    
    # 2. Extract the exact center and surface normal
    centroid = np.mean(points, axis=0)
    
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    
    # Ensure normal points outward (towards the camera, assumed near origin)
    if np.dot(normal, centroid) > 0:
        normal = -normal

    # Verify your approach angle constraints
    if abs(normal[2]) > 1e-6:
        angle_x = np.arctan(abs(normal[0] / normal[2]))
        angle_y = np.arctan(abs(normal[1] / normal[2]))
        if angle_x > theta_allowed or angle_y > theta_allowed:
            return None, "Largest plane violates allowed approach angle constraints."
            
    # 3. Find the longest axis of the plane (the diagonal/length) using PCA
    cov_matrix = np.cov(points.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
    
    # In 3D PCA, eigenvectors[:, 2] corresponds to the largest variance (longest axis)
    major_axis = eigenvectors[:, 2]
    
    # Force the major axis to be perfectly orthogonal to the plane normal
    major_axis = major_axis - np.dot(major_axis, normal) * normal
    major_axis = major_axis / np.linalg.norm(major_axis)

    # 4. Calculate cup positions centered on the plane, spread along the longest axis
    cup1_pos = centroid + (dist_between_cups / 2.0) * major_axis
    cup2_pos = centroid - (dist_between_cups / 2.0) * major_axis

    # 5. Verify the cups are safely on the plane (away from edges)
    # We search ONLY within the flat inlier cloud. If a cup is hanging off the edge 
    # or over a curved corner, it will fail to find enough flat support points.
    kdtree = o3d.geometry.KDTreeFlann(inlier_cloud)
    [k1, _, _] = kdtree.search_radius_vector_3d(cup1_pos, cup_radius)
    [k2, _, _] = kdtree.search_radius_vector_3d(cup2_pos, cup_radius)

    if k1 >= min_points_for_seal and k2 >= min_points_for_seal:
        best_grasp = {
            "center": centroid,
            "normal": normal,
            "cup1": cup1_pos,
            "cup2": cup2_pos,
            "score": float(len(inliers)), # Score is just the sheer number of valid flat points
            "inliers": inliers,           # ADDED FOR DIAGRAM 1 & 2
            "major_axis": major_axis      # ADDED FOR DIAGRAM 2
        }
        return best_grasp, "Found stable planar grasp."
    else:
        return None, f"Cups overhang the flat edges. Point counts: {k1}, {k2}"