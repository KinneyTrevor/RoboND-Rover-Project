import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
#%matplotlib inline
import cv2
import glob

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1

    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    #image = mpimg.imread(binary_img)
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Rotation here
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)

    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    
    # Apply a scaling and a translation
    scale = 10
    x_world = np.int_(xpos + (xpix_rot / scale))
    y_world = np.int_(ypos + (ypix_rot / scale))
    world_size = 200
    x_pix_world = np.clip(x_world, 0, world_size - 1)
    y_pix_world = np.clip(y_world, 0, world_size - 1)
    xpix_translated = 0
    ypix_translated = 0
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    #imagepath = "C:\\Users\\Trevor\\Documents\\GitHub\\RoboND-Rover-Project\\imgfolder\\*"
    #imagepath = "C:\\Users\\The Commander\\Documents\\GitHub\\RoboND-Rover-Project\\imgfolder\\IMG\\*"
    #image_list = glob.glob(imagepath) # Grab Image
    img = Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14,140],[301,140],[200,96],[118,96]])
    destination = np.float32([[img.shape[1] / 2 - dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1] /2 + dst_size, img.shape[0] - bottom_offset],
                              [img.shape[1] / 2 - dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
                              [img.shape[1] / 2 + dst_size, img.shape[0] - 2 * dst_size - bottom_offset],
                             ])
    # 2) Apply perspective transform
    warped = perspect_transform(img,source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed_terrain = color_thresh(warped)
    threshed_obstacles = color_thresh(warped, (100, 100, 100))
    threshed_rocks = color_thresh(warped, (200, 200, 20))
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = threshed_obstacles       
    Rover.vision_image[:,:,1] = threshed_rocks
    Rover.vision_image[:,:,2] = threshed_terrain*255

    # 5) Convert map image pixel values to rover-centric coords
    ter_x, ter_y = rover_coords(threshed_terrain)
    obst_x, obst_y = rover_coords(threshed_obstacles)
    rock_x,rock_y = rover_coords(threshed_rocks)
    # 6) Convert rover-centric pixel values to world coordinates
    (xpos, ypos) = Rover.pos
    yaw = Rover.yaw
    world = Rover.worldmap.shape[0]
    scale = dst_size * 2
    ter_world_x, ter_world_y = pix_to_world(ter_x, ter_y, xpos, ypos, yaw, world, scale)
    obst_world_x, obst_world_y = pix_to_world(obst_x, obst_y, xpos, ypos, yaw, world, scale)
    rock_world_x, rock_world_y = pix_to_world(rock_x, rock_y, xpos, ypos, yaw, world, scale)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    Rover.worldmap[obst_world_y, obst_world_x, 0] += 1
    Rover.worldmap[rock_world_y, rock_world_x, 1] += 1
    Rover.worldmap[ter_world_y, ter_world_x, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    dist, angles = to_polar_coords(ter_x, ter_y)
    Rover.nav_dists = dist
    Rover.nav_angles = angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    
 
    
    
    return Rover
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        