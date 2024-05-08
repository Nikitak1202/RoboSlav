import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

 
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
            
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
            
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
            
    return roll_x, pitch_y, yaw_z # in radians
 

def ArucoDetector(frame, aruco_side_length, aruco_dictionary, camera_calibration_parameters_filename): 
    detected = False
    translation_x, translation_z, pitch_y = 0, 0, 0

    # Load the camera parameters from the saved file
    cv_file = cv2.FileStorage(camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
    mtx = cv_file.getNode('K').mat()
    dst = cv_file.getNode('D').mat()
    cv_file.release()

    # Detect ArUco markers in the video frame
    corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, aruco_dictionary, mtx, dst)
             
    # Check that at least one ArUco marker was detected
    if marker_ids is not None:
        detected = True
        # Draw a square around detected markers in the video frame
        cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
             
        # Get the rotation and translation vectors
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, aruco_side_length, mtx, dst)
                 
        # Print the pose for the ArUco marker
        # The pose of the marker is with respect to the camera lens frame.
        # Imagine you are looking through the camera viewfinder, 
        # the camera lens frame's:
        # x-axis points to the right
        # y-axis points straight down towards your toes
        # z-axis points straight ahead away from your eye, out of the camera

        # Store the translation (i.e. position) information
        translation_x = tvecs[0][0][0]
        translation_y = tvecs[0][0][1]
        translation_z = tvecs[0][0][2]
 
        # Store the rotation information
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[0][0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()     
                 
        # Quaternion format         
        rotation_x = quat[0] 
        rotation_y = quat[1] 
        rotation_z = quat[2] 
        rotation_w = quat[3] 
                 
        # Euler angle format in radians
        roll_x, pitch_y, yaw_z = euler_from_quaternion(rotation_x,
                                                       rotation_y,
                                                       rotation_z,
                                                       rotation_w)
                 
        roll_x_deg = math.degrees(roll_x)
        pitch_y_deg = math.degrees(pitch_y)
        yaw_z_deg = math.degrees(yaw_z)
        print("translation_x: {}".format(translation_x))
        print("translation_y: {}".format(translation_y))
        print("translation_z: {}".format(translation_z))
        print("roll_x: {}".format(roll_x_deg))
        print("pitch_y: {}".format(pitch_y_deg))
        print("yaw_z: {}".format(yaw_z_deg))
        print()

    return detected, translation_x - math.sin(pitch_y), translation_z - math.cos(pitch_y), pitch_y


cap = cv2.VideoCapture(1)
_, frame = cap.read()   
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
detected, goal_x, goal_y, angle_dif = ArucoDetector(frame, 0.13, aruco_dict, 'calibration_chessboard.yaml')
print("X: "+str(goal_x))
print("Y: "+str(goal_y))
print("Angle: "+str(angle_dif))