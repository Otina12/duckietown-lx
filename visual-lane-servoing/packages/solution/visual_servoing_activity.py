# from typing import Tuple
# import numpy as np
# import cv2

from typing import Tuple

import numpy as np
import cv2


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:

    # TODO: write your function instead of this one
    steer_matrix_left = np.zeros(shape)
    steer_matrix_left[230:, 100:250] = -1
    
    return steer_matrix_left


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:

 # TODO: write your function instead of this one
    steer_matrix_right = np.zeros(shape)
    steer_matrix_right[220:, 400:680] = 1

    return steer_matrix_right


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    imgbgr = image
    img = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HSV)
    imgg = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)

    mask_ground = np.ones(imgg.shape, dtype=np.uint8)
    mask_ground[0:180, ::] = 0

    white_lower_hsv = np.array([0, 0, 150])
    white_upper_hsv = np.array([180, 50, 255])
    yellow_lower_hsv = np.array([20, 45, 150])
    yellow_upper_hsv = np.array([40, 255, 255])


    mask_white = cv2.inRange(img, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(img, yellow_lower_hsv, yellow_upper_hsv)

    mask_left_edge = mask_ground * mask_yellow 
    mask_right_edge = mask_ground * mask_white


    return mask_left_edge, mask_right_edge

# def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
#     """
#     Args:
#         shape:              The shape of the steer matrix.
#     Return:
#         steer_matrix_left:  The steering (angular rate) matrix for Braitenberg-like control
#                             using the masked left lane markings (numpy.ndarray)
#     """
#     # TODO: implement your own solution here
#     # steer_matrix_left = np.random.rand(*shape)
#     # ---
#     return get_mask(shape, "left")

# def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
#     """
#     Args: 
#     shape:               The shape of the steer matrix.

#     Return:
#     steer_matrix_right:  The steering (angular rate) matrix for Braitenberg-like control
#                              using the masked right lane markings (numpy.ndarray)
#     """
#     # TODO: implement your own solution here
#     # steer_matrix_right = np.random.rand(*shape)
#     # ---
#     return get_mask(shape, "right")

# def get_symmetry(P, W):
#     Q = []
#     for (x,y) in P:
#         Q.append((x,W-y))
#     return Q

# def get_mask(shape: Tuple[int, int], side: str):
    
#     EVAL_CALIB_CONST = 300 #https://github.com/duckietown/duckietown-lx-recipes/blob/mooc2022/visual-lane-servoing/assets/environment/challenges/scenarios/sampled/LF-small-loop-000/scenario.yaml#L2
#     SIM_CALIB_CONST_RIGHT = 74000  #  
#     SIM_CALIB_CONST_LEFT =  66700 # from calibration during simulation exercise
    
#     assert side in ["right","left"], "ERROR: side not correct"
#     height, width = shape
#     x, y = np.mgrid[:height, :width]
#     coors=np.hstack((x.reshape(-1, 1), y.reshape(-1,1))) 

#     # Duckiebot
#     # pts_left = [(1.0*height, 0.07*width), 
#     #             (0.45*height, 0.45*width), 
#     #             (0.45*height, 0.5*width), 
#     #             (1*height, 0.5*width), 
#     #             (1*height, 0.02*width)] 
#     # Simulation
#     pts_left = [(0.89*height, 0.03*width), 
#                 (0.56*height, 0.26*width), 
#                 (0.56*height, 1.0*width), 
#                 (0.89*height, 1.0*width), 
#                 (0.89*height, 0.0*width)] 
            
#     mask_Left = Path(pts_left).contains_points(coors) * -0.0037 #(EVAL_CALIB_CONST / SIM_CALIB_CONST_LEFT) 

#     # Points zone right: symmetric to zone1
#     pts_right = get_symmetry(pts_left, width)
#     mask_Right = Path(pts_right).contains_points(coors) * 0.0031 # (EVAL_CALIB_CONST / SIM_CALIB_CONST_RIGHT) 

#     if side == "left":
#         return mask_Left.reshape(height, width)
#     elif side == "right":
#         return mask_Right.reshape(height, width)


# def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
#     h, w, _ = image.shape

#     imgbgr = image
#     img_hsv = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2HSV)
#     img_gray = cv2.cvtColor(imgbgr, cv2.COLOR_BGR2GRAY)

#     mask_ground = np.ones(img_gray.shape, dtype=np.uint8)
#     mask_ground[0:h//2, :] = 0

#     white_lower_hsv = np.array([0, 0, 150])
#     white_upper_hsv = np.array([180, 50, 255])
#     yellow_lower_hsv = np.array([15, 45, 145])
#     yellow_upper_hsv = np.array([45, 255, 255])

#     mask_white = cv2.inRange(img_hsv, white_lower_hsv, white_upper_hsv)
#     mask_yellow = cv2.inRange(img_hsv, yellow_lower_hsv, yellow_upper_hsv)

#     mask_left_edge = mask_ground * mask_yellow 
#     mask_right_edge = mask_ground * mask_white

#     mask_left_edge[:, w//3:] = 0
#     mask_right_edge[:, 0:w//3] = 0

#     return mask_left_edge, mask_right_edge


    