import cv2
import cv2.aruco as aruco
import numpy as np
import rotm2euler
import matplotlib.pyplot as plt
import math
import os
import time
import GUI
import aruco_axis
import argparse

# Command line arguments
parser = argparse.ArgumentParser(description='Implements the Position-Based Visual Servo.')
parser.add_argument('-sc', '--show_charts', type=bool, action=argparse.BooleanOptionalAction,
                    metavar='', required=False, help='Shows the charts of position and orientation in real time')
parser.add_argument('-sg', '--show_gui', default=True, type=bool,
                    action=argparse.BooleanOptionalAction, metavar='', required=False,
                    help='Shows the gui to reach the desired pose of the ArUco marker')
args = parser.parse_args()

MARKER_SIZE = 95  # millimeters
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)

def main():
    # Load intrinsic parameters of the camera
    with np.load('bin/camera_matrix.npz') as X:
        K, dist, rvecs, tvecs = [X[i] for i in ('camera_matrix', 'dist', 'rvecs', 'tvecs')]

    # Define the 4X4 bit ArUco tag dictionary
    ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    # Set up the camera
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    # Define coordinates in object coordinate space (3D space)
    obj_points = np.zeros((5, 3), np.float32)
    obj_points[1, 0], obj_points[1, 1], obj_points[2, 0] = -MARKER_SIZE / 2, -MARKER_SIZE / 2, MARKER_SIZE / 2
    obj_points[2, 1], obj_points[3, 0], obj_points[3, 1] = -MARKER_SIZE / 2, MARKER_SIZE / 2, MARKER_SIZE / 2
    obj_points[4, 0], obj_points[4, 1] = -MARKER_SIZE / 2, MARKER_SIZE / 2

    # 3D axis coordinates to be drawn on the ArUco marker
    axis = np.float32([[45, 0, 0], [0, -45, 0], [0, 0, -45]]).reshape(-1, 3)

    # Initialize pose matrices and lists
    estimated_pose = np.identity(n=4, dtype=np.float64)
    desired_pose = np.identity(n=4, dtype=np.float64)
    roll_list, pitch_list, yaw_list = [], [], []
    x_list, y_list, z_list = [], [], []
    x_e_list, y_e_list, z_e_list = [], [], []
    roll_e_list, pitch_e_list, yaw_e_list = [], [], []
    time_list = []

    if args.show_charts:
        figure1, ax1 = plt.subplots(nrows=2, ncols=1, figsize=(7, 5))
        figure2, ax2 = plt.subplots(nrows=2, ncols=1, figsize=(7, 5))
        start_time = time.time()

    if args.show_gui:
        # Set up the GUI window to display the info
        root = 'PBVS - info'
        cv2.namedWindow(root)
        img_info = np.ones((600, 700, 3), np.uint8)

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if args.show_gui:
            cv2.namedWindow(root)
            img_info = np.ones((600, 700, 3), np.uint8)

        # Convert frame to grayscale for detection
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(frame_gray, ARUCO_DICT, K, dist)
        
        # Check if at least one marker is detected
        if len(corners) > 0 and ids is not None:
            # Load the desired pose if available
            try:
                desired_pose = np.load('bin/desired_pose.npy')
                desired_realworld_tvec = np.load('bin/desired_realworld_tvec.npy')
                desired_euler_angles = np.load('bin/desired_euler_angles.npy')
            except FileNotFoundError:
                print('[INFO]: FileNotFoundError handled, check if all .npy files were loaded')

            aruco.drawDetectedMarkers(frame, corners)

            # Loop through each detected marker
            for marker_idx in range(len(ids)):
                # Calculate center of marker
                aruco_center = np.asarray((abs(corners[marker_idx][0][2][0] + corners[marker_idx][0][0][0]) // 2,
                                           abs(corners[marker_idx][0][2][1] + corners[marker_idx][0][0][1]) // 2)).astype(int)

                # Array with the center of the ArUco marker
                new_corners = np.array([np.vstack((aruco_center, corners[marker_idx][0]))])

                try:
                    # SolvePnP to find rotation and translation vectors
                    _, rvec, tvec = cv2.solvePnP(obj_points, new_corners[0], K, dist)

                    # Project the 3D axis points to the image plane
                    img_pts, jac = cv2.projectPoints(axis, rvec, tvec, K, dist)
                    aruco_axis.draw_axis(frame, new_corners[0], img_pts)

                    # Transform rvec to rotation matrix, then to Euler angles
                    rvec_flipped = -rvec.ravel()
                    tvec_flipped = -tvec.ravel()
                    R, jacobian = cv2.Rodrigues(rvec_flipped)
                    realworld_tvec = np.dot(R, tvec_flipped)
                    realworld_tvec[1], realworld_tvec[2] = -realworld_tvec[1], -realworld_tvec[2]
                    pitch, roll, yaw = rotm2euler.rotation_matrix_to_euler_angles(R)
                    pitch, roll, yaw = math.degrees(pitch), math.degrees(roll), math.degrees(yaw)

                    # Display GUI information for each marker
                    if args.show_gui:
                        GUI.display_info_on_screen(img=frame, tvec=realworld_tvec, euler=[roll, pitch, yaw],
                                                   tvec_d=desired_realworld_tvec, euler_d=desired_euler_angles)

                    # Append data to lists for charts if showing charts
                    if args.show_charts:
                        x_list.append(realworld_tvec[0])
                        y_list.append(realworld_tvec[1])
                        z_list.append(realworld_tvec[2])
                        roll_list.append(roll)
                        pitch_list.append(pitch)
                        yaw_list.append(yaw)

                except IndexError:
                    print('[INFO]: IndexError handled')

            if args.show_charts:
                current_time = time.time() - start_time
                time_list.append(current_time)
                GUI.display_pose_graphs(time=time_list, current_time=current_time, x=x_list, y=y_list,
                                        z=z_list, R=roll_list, P=pitch_list, Y=yaw_list, axis=ax1)
                GUI.display_error_graphs(time=time_list, current_time=current_time, x_e=x_e_list,
                                         y_e=y_e_list, z_e=z_e_list, R_e=roll_e_list, P_e=pitch_e_list, Y_e=yaw_e_list,
                                         axis=ax2)

            plt.pause(0.001)  # Refresh the plot

            if args.show_gui:
                GUI.display_translation_info(img=img_info, tvec=realworld_tvec, tvec_d=desired_realworld_tvec)
                GUI.display_rotation_info(img=img_info, euler=[roll, pitch, yaw], euler_d=desired_euler_angles)
                GUI.display_interpretation(img=img_info, tvec=realworld_tvec, euler=[roll, pitch, yaw],
                                           tvec_d=desired_realworld_tvec, euler_d=desired_euler_angles)


            # Calculate the necessary movements to achieve the desired pose
            x_error = realworld_tvec[0] - desired_realworld_tvec[0]
            y_error = realworld_tvec[1] - desired_realworld_tvec[1]
            z_error = realworld_tvec[2] - desired_realworld_tvec[2]

            roll_error = roll - desired_euler_angles[0]
            pitch_error = pitch - desired_euler_angles[1]
            yaw_error = yaw - desired_euler_angles[2]

            # Print translation and rotation errors as a proof of concept
            print("Translation Errors (in mm):")
            print(f"  x: {x_error:.2f}, y: {y_error:.2f}, z: {z_error:.2f}")

            print("Rotation Errors (in degrees):")
            print(f"  Roll: {roll_error:.2f}, Pitch: {pitch_error:.2f}, Yaw: {yaw_error:.2f}")

        
        if args.show_gui:
            GUI.display_background(img_info)
            cv2.imshow(root, img_info)

        # Display main frame
        cv2.imshow('PVBS - RGB', frame)

        # Save current pose if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            desired_pose = estimated_pose
            desired_euler_angles = np.array([roll, pitch, yaw])
            desired_realworld_tvec = realworld_tvec

            np.save('bin/desired_pose.npy', desired_pose)
            np.save('bin/desired_euler_angles.npy', desired_euler_angles)
            np.save('bin/desired_realworld_tvec.npy', desired_realworld_tvec)
            print('[INFO]: ArUco marker pose saved')

        # Exit on ESC key
        if cv2.waitKey(1) & 0xFF == 27:
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
