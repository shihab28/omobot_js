# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import cv2, os

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""













def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=5,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )



















def show_camera():
    window_title = "CSI Camera _ press P to capture"

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=2))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    ind = 0

    if video_capture.isOpened():
        try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, frame)
                else:
                    break 
                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                if keyCode == ord('p') or keyCode == ord('P'):
                    cv2.imwrite("/home/shihab/omobot_js/camera_calibrator/images/images_{}.jpg".format(ind), frame)
                    ind += 1
                elif keyCode == 27 or keyCode == ord('q'):
                    break

        finally:
            video_capture.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")

def show_stereo_camera():
    window_title_0 = "CSI Camera0 _ press M to Singlecapture, B for both"
    window_title_1 = "CSI Camera1 _ press N to Singlecapture, B for both"

    video_capture_0 = cv2.VideoCapture(gstreamer_pipeline(0, flip_method=0), cv2.CAP_GSTREAMER)
    video_capture_1 = cv2.VideoCapture(gstreamer_pipeline(1, flip_method=0), cv2.CAP_GSTREAMER)

    ind0 = 0
    ind1 = 0
    ind = 0

    if not os.path.isdir("/home/shihab/omobot_js/camera_calibrator/images/left"):
        os.makedirs("/home/shihab/omobot_js/camera_calibrator/images/left")
    if not os.path.isdir("/home/shihab/omobot_js/camera_calibrator/images/rght"):
        os.makedirs("/home/shihab/omobot_js/camera_calibrator/images/rght")

    if video_capture_0.isOpened() and video_capture_1.isOpened():
        try:
            window_handle_0 = cv2.namedWindow(window_title_0, cv2.WINDOW_AUTOSIZE)
            window_handle_1 = cv2.namedWindow(window_title_1, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val_0, frame_0 = video_capture_0.read()
                ret_val_1, frame_1 = video_capture_1.read()


                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                
                if cv2.getWindowProperty(window_title_0, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title_0, frame_0)
                else:
                    break 
                    
                if cv2.getWindowProperty(window_title_1, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title_1, frame_1)
                else:
                    break 

                keyCode = cv2.waitKey(10) & 0xFF
                 
                if keyCode == ord('b') or keyCode == ord('B'):
                    cv2.imwrite("/home/shihab/omobot_js/camera_calibrator/images/left/images_{}.jpg".format(ind), frame_0)
                    cv2.imwrite("/home/shihab/omobot_js/camera_calibrator/images/rght/images_{}.jpg".format(ind), frame_1)
                    print("saved /home/shihab/omobot_js/camera_calibrator/images/left_right/images_{}.jpg".format(ind))
                    ind += 1

                if keyCode == ord('m') or keyCode == ord('M'):
                    cv2.imwrite("/home/shihab/omobot_js/camera_calibrator/images/left/images_{}.jpg".format(ind0), frame_0)
                    print("saved /home/shihab/omobot_js/camera_calibrator/images/left/images_{}.jpg".format(ind0))
                    ind0 += 1
                elif keyCode == ord('n') or keyCode == ord('N'):
                    cv2.imwrite("/home/shihab/omobot_js/camera_calibrator/images/rght/images_{}.jpg".format(ind1), frame_1)
                    print("saved /home/shihab/omobot_js/camera_calibrator/images/rght/images_{}.jpg".format(ind1))
                    ind1 += 1
                elif keyCode == 27 or keyCode == ord('q'):
                    break

        finally:
            video_capture_0.release()
            video_capture_1.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    # show_camera()
    show_stereo_camera()
    
