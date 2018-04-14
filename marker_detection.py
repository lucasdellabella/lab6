# Lucas Della Bella
# Saqib Ali

from skimage import io, feature, color, transform, filters, morphology, segmentation, measure, exposure, util
import numpy as np
import random
import os
import math

def image_for_filename(image_name):
    img = io.imread(image_name)
    img = color.rgb2gray(img)
    return img

def binary_image(image):
    return (image < filters.threshold_mean(image))

TEMPLATE_BOTTOM_LEFT = binary_image(image_for_filename('marker-template-bottom-left.png'))
TEMPLATE_TOP_RIGHT = binary_image(image_for_filename('marker-template-top-right.png'))

def unwarp_marker(image, left_region, right_region, width=320, height=240, margin=20):
    '''
    Using the four corners of the detected marker, estimate the inverse
    projective transform to obtain a cropped, "unwarped" view of the marker

    Input: 
        - image: grayscale image that contains marker
        - left_region: regionprops corresponding to lower-left corner
        - right_region: regionprops corresponding to upper-right corner
        - width, height: output "unwarped" image dimensions
        - margin: increase the "unwarped" bounding box by the given margin
    Returns: the "unwarped" image that contains just the marker
    '''
    
    li = left_region.intensity_image
    ri = right_region.intensity_image

    left_miny, left_minx, left_maxy, left_maxx = left_region.bbox
    right_miny, right_minx, right_maxy, right_maxx = right_region.bbox

    # Compute the coordinates of the corners
    top_right = (right_maxx, right_miny)
    bottom_left = (left_minx, left_maxy)
    
    # Currently, the corners of the image essentially correspond to the bounding box,
    # which means that the "unwarped" image is essentially just a crop
    # TODO: detect the lines of the marker to estimate actual corners for better "unwarping"
    top_left = (bottom_left[0], top_right[1])
    bottom_right = (top_right[0], bottom_left[1])

    # Estimate the transform
    m = margin
    src = np.array([
        [m, m],
        [width - m, m],
        [m, height - m],
        [width - m, height - m]    
    ])
    dst = np.array([
        top_left,      # top left     -> (m, m)
        top_right,     # top right    -> (width - m, m)
        bottom_left,   # bottom left  -> (m, height - m)
        bottom_right   # bottom right -> (width - m, height - m)
    ])
    
    t = transform.ProjectiveTransform()
    t.estimate(src, dst)
    
    return transform.warp(image, t, output_shape=(height, width), mode='constant', cval=1.0)

def overlap_measure(image1, image2):
    '''
    Measures the ratio of pixels common to both images of the same size
    
    Inputs: two images of identical dimensions
    Returns: ratio of pixels in common
    '''

    overlap_area = np.count_nonzero(np.equal(image1, image2))
    total_area = image1.size
    
    return overlap_area / total_area 

def region_filter_heuristic(region, orientation_deviation=15, overlap_minimum=0.8):
    '''
    For a given region, determines whether to consider it as a possible marker corner
    using a variety of factors:
        - large enough area
        - closeness to 45 degree orientation
        - similarity to corner templates

    Inputs:
        - region: regionprops to determine whether to consider or not
        - orientation_deviation: allowed deviation from 45 degree orientation for inclusion
        - overlap_minimum: minimum template similarity measure for inclusion
    Returns: true if region should be considered, false otherwise
    '''
    orientation = np.rad2deg(abs(region.orientation))
    area = region.area
    
    # Area must be large enough
    if area < 100:
        return False
    
    # The markers should have orientations close to 45 degrees
    if orientation < (45 - orientation_deviation) or orientation > (45 + orientation_deviation):
        return False
    
    # The markers should look like the templates
    region_image = region.intensity_image
    template_left = transform.resize(TEMPLATE_BOTTOM_LEFT, region_image.shape)
    template_right = transform.resize(TEMPLATE_TOP_RIGHT, region_image.shape)  
    
    # Compute the overlap measure for both template images
    overlap_left = overlap_measure(region_image, template_left)
    overlap_right = overlap_measure(region_image, template_right)
    overlap_ratio = max(overlap_left, overlap_right)
    
    if overlap_ratio < overlap_minimum:
        return False
    
    return True

def select_best_candidate(regions):
    '''
    Returns the pair of regions determined to be the corners of the marker

    Input: list of (filtered) regionprops corresponding to possible marker corners
    Returns: 
        returns a 2-tuple of (left corner region, right corner region) if a 
        candidate pair of corners is found; otherwise, returns an empty tuple
    '''
    # Select the two best marker regions  
    best_candidates = ()
    best_measure = 0
    
    # Compare each pair of regions
    for i in range(len(regions)):
        for j in range(i + 1, len(regions)):
            region_i = regions[i]
            region_j = regions[j]
            
            # Determine which region is the left and which is the right
            if region_i.bbox[1] < region_j.bbox[1]:
                region_left = region_i
                region_right = region_j
            else:
                region_left = region_j
                region_right = region_i
                
            # Left region should be strictly left+lower of the right region
            rl_min_y, rl_min_x, rl_max_y, rl_max_x = region_left.bbox
            rr_min_y, rr_min_x, rr_max_y, rr_max_x = region_right.bbox
            
            if rl_max_x > rr_min_x or rl_min_y < rr_max_y:
                continue
            
            # Areas of both regions should be similar (closer to 1 means identical area)
            area_measure = min(region_left.area, region_right.area) / max(region_left.area, region_right.area)
            
            # Regions should be similar to the templates
            image_left = region_left.intensity_image
            image_right = region_right.intensity_image
            template_left = transform.resize(TEMPLATE_BOTTOM_LEFT, image_left.shape)
            template_right = transform.resize(TEMPLATE_TOP_RIGHT, image_right.shape)
            
            overlap_left_measure = overlap_measure(image_left, template_left)
            overlap_right_measure = overlap_measure(image_right, template_right)
            
            # If any overlap measure is too low, then false marker possibly detected
            if overlap_left_measure < 0.5 or overlap_right_measure < 0.5:
                continue
            
            # Closer to 1 is better
            pair_measure = area_measure * overlap_left_measure * overlap_right_measure
            
            if pair_measure > best_measure:
                best_candidates = (region_left, region_right)
                best_measure = pair_measure
                
    return best_candidates


def process_regions(image, blur_sigma=3, opening_size=3, orientation_deviation=15, overlap_minimum=0.8):
    '''
    Attempt to find any possible marker corner regions in a given image

    Inputs:
        - image: grayscale image that may contain a marker
        - blur_sigma: parameter for Gaussian blur to use on image
        - opening_size: parameter for morphological opening to use on image
        - orientation_deviation: see orientation parameter used by region_filter_heuristic(...)
        - overlap_minimum: see similarity parameter used by region_filter_heuristic(...)
    Returns: a 2-tuple of:
        - the image after pre-processing steps like blurring, thresholding, etc.
        - the list of regionprops that may be possible marker corners
    '''
    # Blur and equalize the image
    image = exposure.equalize_hist(image)
    image = filters.gaussian(image, sigma=blur_sigma)
    
    # Use local thresholding
    image = (image <= filters.threshold_sauvola(image, k=0.1))
    image = morphology.opening(image, selem=morphology.disk(opening_size))
    
    # Label components in the image
    labeled = measure.label(image, connectivity=2)
    components = measure.regionprops(labeled, intensity_image=image)
    overlay = color.label2rgb(labeled, image)
    
    # Sort the components by our rectangle heuristic
    return image, [r for r in components if region_filter_heuristic(r, orientation_deviation, overlap_minimum)]


def detect_marker(image):
    '''
    Attempts to detect a marker in the given grayscale image.

    Input: grayscale image that may contain a marker
    Returns: a marker detection dict that contains the following properties:
        {
            'detected': bool, whether a marker was found

            # If 'detected' is True:
                'marker_width': width in pixels of the detected marker
                'unwarped_image': (400x300) image cropped to contain just the marker
                'left_corner_region': regionprops for the detected lower-left corner
                'right_corner_region': regionprops for the detected upper-right corner

            # Diagnostic properties for debugging are always available
                'regions': list of regionprops, all the candidate regions in the image
                'filtered_image': image after processing (blurs, threshold, etc.)
        }
    '''
    # Obtain the possible regions for the markers in the image
    filtered_image, marker_regions = process_regions(image)
    
    # If two components were not found after processing, relax the requirements
    if len(marker_regions) < 2:
        blur_sigma = 2
        opening_size = 2
        orientation_deviation = 25
        overlap_minimum = 0.65
        
        filtered_image, marker_regions = process_regions(image, blur_sigma, opening_size, orientation_deviation, overlap_minimum)
        
    # Keep the rectangle regions (sorted left to right)    
    candidates = select_best_candidate(marker_regions)
    
    if candidates:
        # Compute properties of the marker
        left, right = candidates
        left_ytop, left_xleft, left_ybottom, left_xright = left.bbox
        right_ytop, right_xleft, right_ybottom, right_xright = right.bbox
        marker_width = right_xright - left_xleft
        unwarped = unwarp_marker(image, left, right)
        
        return {
            'detected': True,
            
            'left_corner_region': left,
            'right_corner_region': right,
            
            'marker_width': marker_width,
            'unwarped_image': unwarped,
            
            # Diagnostic properties
            'regions': marker_regions,
            'filtered_image': filtered_image,
        }
        
    return { 
        'detected': False,
        
        # Diagnostic properties
        'regions': marker_regions,
        'filtered_image': filtered_image,
    } 