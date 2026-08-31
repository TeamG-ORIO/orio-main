import numpy as np
import open3d as o3d

def compute_mask_com(mask):
    ys, xs = np.where(mask)

    if len(xs) == 0:
        return None

    return (float(np.median(xs)), float(np.median(ys)))


def optimize_grasp_pose(pcd, cup_radius, dist_between_cups, theta_allowed, min_points_for_seal,
                        gripper_extent=[0.08, 0.04, 0.06], base_standoff=0.03, max_shift=0.05):
    """
    Deterministic grasp calculator for planar objects.
    Extracts the largest plane, centers on it, aligns with the longest axis, 
    and checks for gripper body collisions, shifting outward if necessary.
    
    Args:
        ... [previous args] ...
        gripper_extent: [x, y, z] width, depth, and height of the rigid gripper body in meters.
        base_standoff: Distance from the suction cup tips to the rigid body in meters.
        max_shift: Maximum distance (meters) the gripper is allowed to pull back to avoid collision.
    """
    if len(pcd.points) < 100:
        return None, "Not enough points in point cloud."

    # 1. Isolate the largest planar surface using RANSAC
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.015,
                                             ransac_n=3,
                                             num_iterations=1000)
    
    if len(inliers) < min_points_for_seal * 2:
        return None, "No planar surface found large enough to grasp."

    inlier_cloud = pcd.select_by_index(inliers)
    points = np.asarray(inlier_cloud.points)
    
    # 2. Extract the exact center and surface normal
    centroid = np.mean(points, axis=0)
    
    [a, b, c, d] = plane_model
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    
    if np.dot(normal, centroid) > 0:
        normal = -normal

    if abs(normal[2]) > 1e-6:
        angle_x = np.arctan(abs(normal[0] / normal[2]))
        angle_y = np.arctan(abs(normal[1] / normal[2]))
        if angle_x > theta_allowed or angle_y > theta_allowed:
            return None, "Largest plane violates allowed approach angle constraints."
            
    # 3. Find the longest axis of the plane (the diagonal/length) using PCA
    cov_matrix = np.cov(points.T)
    eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
    
    major_axis = eigenvectors[:, 2]
    major_axis = major_axis - np.dot(major_axis, normal) * normal
    major_axis = major_axis / np.linalg.norm(major_axis)

    # 4. Calculate cup positions centered on the plane
    cup1_pos = centroid + (dist_between_cups / 2.0) * major_axis
    cup2_pos = centroid - (dist_between_cups / 2.0) * major_axis

    # 5. Verify the cups are safely on the plane
    kdtree = o3d.geometry.KDTreeFlann(inlier_cloud)
    [k1, _, _] = kdtree.search_radius_vector_3d(cup1_pos, cup_radius)
    [k2, _, _] = kdtree.search_radius_vector_3d(cup2_pos, cup_radius)

    if k1 < min_points_for_seal or k2 < min_points_for_seal:
         return None, f"Cups overhang the flat edges. Point counts: {k1}, {k2}"

    # ---------------------------------------------------------
    # 6. COLLISION CHECKING & OUTWARD SHIFTING
    # ---------------------------------------------------------
    
    # Extract dimensions based on vis_tools.py
    stem_w = 0.01
    stem_h = 0.04
    crossbar_h = 0.01
    link_h = 0.02
    disk_thickness = 0.005
    
    # Calculate exact bounding box for the BLUE parts
    gripper_extent = np.array([
        dist_between_cups + stem_w,     # X Width (0.06)
        stem_w * 1.5,                   # Y Depth (0.015)
        stem_h + crossbar_h + link_h    # Z Height (0.07)
    ])
    
    base_standoff = disk_thickness      # Starts immediately after the green disk
    max_shift = 0.05                    # Max distance to pull back
    
    # Build a full 3x3 rotation matrix for the gripper
    z_axis = normal
    x_axis = major_axis
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])

    # Step backward along the normal until clear (in 5mm increments)
    step_size = 0.005 
    
    for shift in np.arange(0.0, max_shift + step_size, step_size):
        # Center of the blue bounding box
        # We move UP the normal by: the shift + the green disk thickness + half the blue height
        z_offset = shift + base_standoff + (gripper_extent[2] / 2.0)
        box_center = centroid + (z_axis * z_offset)
        
        gripper_box = o3d.geometry.OrientedBoundingBox(
            center=box_center,
            R=rotation_matrix,
            extent=gripper_extent
        )
        gripper_box.color = (1.0, 0.0, 0.0) # Color the collision box RED for debugging
        
        # Crop the main point cloud to see what is inside the box
        colliding_points = pcd.crop(gripper_box)
        
        # If 5 points or fewer are inside, we consider it noise and call it safe
        if len(colliding_points.points) <= 5:
            best_grasp = {
                "center": centroid + (z_axis * shift),  # Shifted TCP
                "normal": normal,
                "rotation": rotation_matrix,
                "cup1": cup1_pos + (z_axis * shift),    # Shifted cup location
                "cup2": cup2_pos + (z_axis * shift),    # Shifted cup location
                "score": float(len(inliers)),
                "shift_applied": shift,
                "safe_box": gripper_box                 # Pass the box out for visualization
            }
            msg = "Found stable planar grasp." if shift == 0 else f"Found grasp, shifted outward by {shift*100:.1f}cm to avoid collision."
            return best_grasp, msg

    return None, f"Could not find a collision-free pose even after shifting {max_shift*100:.1f}cm outward."