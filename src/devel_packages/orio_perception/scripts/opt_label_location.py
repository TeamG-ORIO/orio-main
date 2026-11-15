#!/usr/bin/env python3

import numpy as np
import scipy as sc
from scipy.ndimage import distance_transform_edt
import os
import matplotlib.pyplot as plt
import skimage as sk
import cv2
from skimage.feature import peak_local_max
from matplotlib.patches import Polygon

def get_object_orientation(im_mask):
    """
    Calculates the orientation of a binary mask using a minimum area bounding box.
    This correctly identifies the edges of squares/rectangles instead of diagonals.
    Returns the angle in degrees.
    """
    # 1. OpenCV expects a uint8 image for contour finding
    mask_uint8 = im_mask.astype(np.uint8)
    if mask_uint8.max() == 1:
        mask_uint8 *= 255 # Ensure it's 0-255, not 0-1
        
    # 2. Find the contours of the mask
    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return 0.0
        
    # 3. Grab the largest contour (ignores tiny noise pixels in the background)
    largest_contour = max(contours, key=cv2.contourArea)
    
    # 4. Get the minimum area bounding box
    # rect is formatted as: ((center_x, center_y), (width, height), angle)
    rect = cv2.minAreaRect(largest_contour)
    
    # Extract the angle
    angle = rect[2]
    
    # Normalize the angle to be strictly positive and under 180 degrees
    # (OpenCV versions differ on whether minAreaRect returns [0, 90) or [-90, 0))
    angle_deg = angle % 180
    
    return angle_deg
    
def compute_grad_mag(im, thresh=100):
    # The perception node passes RGB but this converts as if BGR (red and blue weights
    # swapped). Kept on purpose: the label placement settings were checked against it on the
    # recorded demo zones, and converting as RGB moved some labels nearer the item's edge.
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    dx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
    dy = cv2.Sobel(gray, cv2.CV_64F, 0, 1)
    mag = cv2.magnitude(dx, dy)
    im_grad = cv2.convertScaleAbs(mag)
    _, im_grad_thresh = cv2.threshold(im_grad, 100, 255, cv2.THRESH_BINARY)
    
    return im_grad, im_grad_thresh

def dist2(x, c):
    
    ndata, dimx = x.shape
    ncenters, dimc = c.shape
    assert dimx == dimc, 'Data dimension does not match dimension of centers'

    return (np.ones((ncenters, 1)) * np.sum((x**2).T, axis=0)).T + \
            np.ones((   ndata, 1)) * np.sum((c**2).T, axis=0)    - \
            2 * np.inner(x, c)


def compute_ANMS(pts, vals, c_rob=0.9):
    N = len(pts)
    if N == 0:
        return np.array([]), np.array([])
        
    min_rads = np.zeros(N)
    dist_mat = dist2(pts, pts)
    
    for i in range(N):
        c = pts[i]
        val_xi = vals[c[0], c[1]]
        
        dists = dist_mat[i]
        dists_idxs = np.argsort(dists)
        sorted_dists = dists[dists_idxs]
        
        radius = np.inf 
        
        for j in range(1, N): 
            sup_pt = pts[dists_idxs[j]]
            val_xj = vals[sup_pt[0], sup_pt[1]]
            
            if val_xi < c_rob * val_xj:
                radius = sorted_dists[j]
                break
                
        min_rads[i] = radius

    best_idxs = np.argsort(-min_rads)
    return pts[best_idxs], min_rads[best_idxs]

def is_safe_oriented_placement(edge_mask, object_mask, center_x, center_y, angle, label_w=120, label_h=40, max_edge_ratio=0.05):
    """
    Evaluates a rotated bounding box to ensure it fits on the object and is clear of edges.
    """
    # 1. Geometrically define the rotated rectangle
    rect = ((float(center_x), float(center_y)), (float(label_w), float(label_h)), float(angle))
    
    # Calculate the 4 corners of the rotated box
    box = cv2.boxPoints(rect)
    box = np.int32(box) 

    # 2. Create a blank mask and draw the label's footprint
    label_footprint = np.zeros_like(object_mask)
    cv2.fillPoly(label_footprint, [box], 255)

    # Calculate the exact pixel area of the rasterized label
    label_area = cv2.countNonZero(label_footprint)
    if label_area == 0:
        return False, 1.0, "Label is entirely off-screen", box

    # 3. INCLUSION CHECK: Does the footprint fall entirely inside the object?
    overlap = cv2.bitwise_and(label_footprint, object_mask)
    overlap_area = cv2.countNonZero(overlap)

    if overlap_area < label_area:
        return False, 1.0, "Label overhangs object boundaries", box

    # 4. CLUTTER CHECK: Look for gradients within the footprint
    # [DELETED THE cv2.threshold LINE HERE]
    
    # Isolate only the edges that fall strictly inside our label footprint using the pre-computed mask
    edges_in_label = cv2.bitwise_and(edge_mask, label_footprint)
    edge_pixel_count = cv2.countNonZero(edges_in_label)

    # Calculate density
    edge_ratio = edge_pixel_count / label_area

    # 5. Evaluate
    is_safe = edge_ratio <= max_edge_ratio

    if is_safe:
        return True, edge_ratio, "Safe for placement", box
    else:
        return False, edge_ratio, "Too much text/clutter", box    

def get_candidate_locs(im, im_mask, alpha=0.2, visualize=False, max_pts=50):
    """max_pts None: every peak, strongest first, without ANMS."""

    # Compute gradient magnitude and threshold it
    im_grad, im_grad_thresh = compute_grad_mag(im)

    # Compute Euclideant distance transform (EDT) of object mask
    mask_edt = distance_transform_edt(im_mask)

    # Compute EDT for gradient threshold image
    grad_flip = 255 - im_grad_thresh
    grad_masked = np.where(im_mask == 1, grad_flip, 0)
    grad_edt = distance_transform_edt(grad_masked)

    # Blend EDT
    blended_edt = alpha * mask_edt + (1-alpha) * grad_edt

    # Find local peaks of blended EDT and perform ANMS
    candidate_pts = peak_local_max(blended_edt, min_distance=10)
    if len(candidate_pts) == 0:
        return im_grad, im_grad_thresh, None
    if max_pts is None:
        best_pts = candidate_pts
    else:
        best_pts, best_rads = compute_ANMS(candidate_pts, blended_edt, c_rob=0.9)

    if visualize:
        # plt.imshow(mask_edt, cmap='gray')
        # plt.show()
        # plt.imshow(grad_edt, cmap='gray')
        # plt.show()
        plt.imshow(grad_masked, cmap='gray')
        plt.show()
        plt.imshow(blended_edt, cmap='gray')
        if len(best_pts) > 0:
            plt.scatter(best_pts[:, 1], best_pts[:, 0], c='red', marker='x', s=5)
        plt.show()
        # print(len(candidate_pts))
        # print(len(best_pts))

    return im_grad, im_grad_thresh, best_pts if max_pts is None else best_pts[:max_pts]


def _outline_clearance(item_edt, box):
    """Smallest distance (px) from the label footprint `box` (4 corners) to the item's
    outline: the minimum of the item's distance transform over the footprint."""
    x0, y0 = np.maximum(box.min(axis=0), 0)
    x1, y1 = np.minimum(box.max(axis=0) + 1, item_edt.shape[::-1])
    if x1 <= x0 or y1 <= y0:
        return 0.0
    fp = np.zeros((y1 - y0, x1 - x0), np.uint8)
    cv2.fillPoly(fp, [box - np.array([x0, y0])], 1)
    vals = item_edt[y0:y1, x0:x1][fp > 0]
    return float(vals.min()) if vals.size else 0.0

def opt_label_loc(im, im_mask, label_zone_num, alpha=0.2, visualize=False, label_w=120, label_h=40,
                  margin_px=0.0, farthest=False, edge_slack=0.02, max_edge_ratio=0.05):
    """label_w, label_h: the label's footprint in pixels (long side, short side).
    margin_px: the label keeps at least this far inside the item's outline. The search runs
    on the mask shrunk by it: the candidate points, both distance maps (to the outline and
    to printed edges) and the fits-inside test all see the shrunk outline.
    farthest: test every candidate point (not the first that fits in ANMS order) and take,
    of the placements that fit, the one farthest from the item's outline, among those whose
    share of edge pixels is at most edge_slack above the cleanest fitting placement's. When
    none fits (every placement inside the item has more than max_edge_ratio of edges), the
    same choice is made among the placements inside the item, whatever their edges; only
    when no candidate lies inside the item does it fall back to the centroid."""
    H, W = im.shape[:2]

    if label_zone_num == 1:
        im = im[:, :W//2]
        im_mask = im_mask[:, :W//2]
    elif label_zone_num == 2:
        im = im[:, W//2:]
        im_mask = im_mask[:, W//2:]
    else:
        # Add proper traceback 
        print(f"Invalid label_zone_num {label_zone_num}")
        return
    
    # 1. Get the major and minor axis angles
    major_axis = get_object_orientation(im_mask)
    minor_axis = (major_axis + 90) % 180
    
    # 2. We only want to check these two specific orientations
    angles_to_check = [major_axis, minor_axis]

    # 3. Calculate the Centroid of the mask for our default fallback
    # Ensure mask is uint8 for cv2.moments
    mask_uint8 = (im_mask * 255).astype(np.uint8) if im_mask.max() == 1 else im_mask.astype(np.uint8)
    M = cv2.moments(mask_uint8)
    
    slice_H, slice_W = im.shape[:2]
    
    # Calculate centroid if the mask isn't completely empty
    if M["m00"] != 0:
        default_x = int(M["m10"] / M["m00"])
        default_y = int(M["m01"] / M["m00"])
    else:
        # Ultimate fallback if mask is totally empty
        default_x, default_y = slice_W // 2, slice_H // 2

    # The mask the label must fit inside: the item shrunk by margin_px (pixels whose distance
    # to the outside is more than margin_px). Orientation and the centroid fallback above
    # come from the whole item.
    if margin_px > 0:
        fit_mask = (distance_transform_edt(im_mask) > margin_px).astype(np.uint8)
    else:
        fit_mask = im_mask

    im_grad, im_grad_thresh, best_pts = get_candidate_locs(im, fit_mask, visualize=visualize,
                                                           max_pts=None if farthest else 50)

    if best_pts is None or len(best_pts) == 0:
        print("No candidate points found in Zone: ", label_zone_num)
        return None, None, None

    # check_pts = np.append([[default_y, default_x]], best_pts, axis=0)
    check_pts = best_pts

    # Initialize defaults using the Centroid and the Minor Axis
    opt_x, opt_y, opt_ang = default_x, default_y, minor_axis 

    final_box = None
    final_msg = "No safe placement found (Using Centroid Fallback)"
    found_safe = False

    if farthest:
        # Every placement inside the item (the shrunk mask), whatever its edges:
        # (edge share, clearance to the outline, x, y, angle, box)
        item_edt = distance_transform_edt(im_mask)
        inside = []
        for y, x in check_pts:
            for check_angle in angles_to_check:
                is_in, ratio, msg, box = is_safe_oriented_placement(
                    im_grad_thresh, fit_mask, float(x), float(y), float(check_angle),
                    label_w=label_w, label_h=label_h, max_edge_ratio=1.0)
                if is_in:
                    inside.append((ratio, _outline_clearance(item_edt, box), x, y, check_angle, box))
        fits = [f for f in inside if f[0] <= max_edge_ratio]
        pool = fits or inside   # nothing clear enough of edges: the cleanest inside the item
        if pool:
            cleanest = min(f[0] for f in pool)
            ok = [f for f in pool if f[0] <= cleanest + edge_slack]
            ratio, _clear, opt_x, opt_y, opt_ang, final_box = max(ok, key=lambda f: (f[1], -f[0]))
            found_safe = bool(fits)
            final_msg = ("Safe for placement (farthest from the outline)" if fits else
                         "No placement under %.0f%% edges: least edges inside the item (%.1f%%)"
                         % (100 * max_edge_ratio, 100 * ratio))
            print(final_msg)
        check_pts = []   # skip the first-fit loop below

    # The Search Loop
    for y, x in check_pts:
        candidates = []
        
        # Iterate directly over the two axis-aligned angles
        for check_angle in angles_to_check: 
            is_safe, ratio, msg, box = is_safe_oriented_placement(
                im_grad_thresh, fit_mask, float(x), float(y), float(check_angle),
                label_w=label_w, label_h=label_h
            )
            
            if is_safe:
                candidates.append((ratio, check_angle, box, msg))

        if len(candidates) > 0:
            candidates.sort() 
            ratio, opt_ang, final_box, final_msg = candidates[0]
            opt_x, opt_y = x, y
            found_safe = True
            break

    if label_zone_num == 2:
        opt_x += W//2
        # opt_y += W//2

    # ====================== Plotting start ==================================== #
    if visualize:
        plt.imshow(im, cmap='gray')
        
        if found_safe:
            color = 'green'
            # Draw the successful box
            label_polygon = Polygon(final_box, closed=True, edgecolor=color, facecolor='none', linewidth=2)
            plt.gca().add_patch(label_polygon)
            plt.title(f"Status: {final_msg} at ({opt_x}, {opt_y}) ang: {opt_ang}")
        else:
            # If no safe spot was found, maybe just plot a red dot at the center default
            plt.scatter([opt_x], [opt_y], c='red', marker='x', s=100)
            plt.title(f"Status: {final_msg}")
            
        plt.show()
    # ====================== Plotting end ===================================== #
    print(f"Axes: {major_axis}, {minor_axis}")
    print(f"Optimal Placement -> X: {opt_x}, Y: {opt_y}, Angle: {opt_ang + 90}")
    return opt_x, opt_y, opt_ang + 90 