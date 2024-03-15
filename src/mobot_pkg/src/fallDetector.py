#!/usr/bin/env /home/shihab/venvs/python3p8/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, os
from datetime import datetime
from ultralytics import YOLO
import numpy as np
import torch
import multiprocessing
from filelock import FileLock
import time

import smtplib
import ssl
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart

mempath = "/dev/shm/"
if os.path.isdir("/dev/shm/") == False:
    os.makedirs(mempath)
    

keynote_indx = {'Nose': 0,
                'Left Eye': 1,
                'Right Eye': 2,
                'Left Ear': 3,
                'Right Ear': 4,
                'Left Shoulder': 5,
                'Right Shoulder': 6,
                'Left Elbow': 7,
                'Right Elbow': 8,
                'Left Wrist': 9,
                'Right Wrist': 10,
                'Left Hip': 11,
                'Right Hip': 12,
                'Left Knee': 13,
                'Right Knee': 14,
                'Left Ankle': 15,
                'Right Ankle': 16}

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

def chng_perspective(img):
    # Ratio 8m width to 6m height (4:3)
    width = 640
    height = 480

    # Input points are read from the corners drawn in the reduced size screenshot provided
    inputpts = np.float32([[31, 95], [505, 89], [516, 220], [18, 228]])
    outputpts = np.float32([[71, 126], [567, 114], [571, 261], [71, 268]])

    m = cv2.getPerspectiveTransform(inputpts, outputpts)
    outimg = cv2.warpPerspective(img, m, (width, height), cv2.INTER_LINEAR)
    return outimg


def fall_detection(box, kpt):
    # [i, cls, *list(*xyxy2xywh(np.array(box)[None])), conf, *list(kpts.detach().cpu().numpy()[index])]
    x1, y1, x2, y2 = box
    dx, dy = x2-x1, y2-y1
    difference = dy - dx
    left_shoulder_x, left_shoulder_y,  lef_sh_conf = kpt[keynote_indx['Left Shoulder']]
    right_shoulder_x, right_shoulder_y, rgt_sh_conf = kpt[keynote_indx['Right Shoulder']]
    left_body_x, left_body_y,     lef_bd_conf = kpt[keynote_indx['Left Hip']]
    right_body_x, right_body_y,     rgt_bd_conf = kpt[keynote_indx['Right Hip']]
    left_foot_x, left_foot_y,     lef_ft_conf = kpt[keynote_indx['Left Ankle']]
    right_foot_x, right_foot_y,     rgt_ft_conf = kpt[keynote_indx['Right Ankle']]

    len_factor = np.math.sqrt(
        ((left_shoulder_y - left_body_y) ** 2 + (left_shoulder_x - left_body_x) ** 2))

    # cond1 = (left_shoulder_y > left_foot_y - len_factor)
    # cond2 = (left_body_y > left_foot_y - (len_factor / 2))
    # cond3 = (left_shoulder_y > left_body_y - (len_factor / 2))
    # cond4 = (right_shoulder_y > right_foot_y - len_factor)
    # cond5 = right_body_y > right_foot_y - (len_factor / 2)
    # cond6 = right_shoulder_y > right_body_y - (len_factor / 2)
    # cond7 = (difference < 0)

    # print(f"LSh:{kpt[keynote_indx['Left Shoulder']]}, Rsh: {kpt[keynote_indx['Right Shoulder']]}")
    # print(f"LBody :{kpt[keynote_indx['Left Hip']]}, RBody: {kpt[keynote_indx['Right Hip']]}")
    # print(f"LFoot :{kpt[keynote_indx['Left Ankle']]}, RFoot: {kpt[keynote_indx['Right Ankle']]}")
    # print(f"Len :{len_factor}")
    # print("Cond" , cond1,cond2 ,cond3 ,cond4 ,cond5 ,cond6 ,cond7 )

    if left_shoulder_y > left_foot_y - len_factor and left_body_y > left_foot_y - (
            len_factor / 2) and left_shoulder_y > left_body_y - (len_factor / 2) or (
            right_shoulder_y > right_foot_y - len_factor and right_body_y > right_foot_y - (
                len_factor / 2) and right_shoulder_y > right_body_y - (len_factor / 2)) \
            or difference < 0:
        return True, (x1, y1, x2, y2)

    # if cond1 and cond2 and cond3 or cond4 and cond5 and cond6 or cond7:
    #     return True, (x1,y1,x2,y2)
    return False, None


def sendEmail(person, ImgFileName):
    time.sleep(1)
    sender_email = os.environ['omobot_email']
    omobot_pass =  os.environ['omobot_pass']
    receiver_email = "shihab.ahamad@maine.edu"
    message = MIMEMultipart("alternative")
    message["Subject"] = "Falling Report"
    message["From"] = sender_email
    message["To"] = receiver_email

    text = """\
    Hi {} has fallen on {}
    """.format(person, datetime.now().strftime("%m/%d/%Y, %H:%M:%S"))
    part1 = MIMEText(text, "plain")
    message.attach(part1)

    if os.path.exists(os.path.abspath(ImgFileName)):
        with open(ImgFileName, 'rb') as f:
            img_data = f.read()
        image = MIMEImage(img_data, name=os.path.basename(ImgFileName))
        message.attach(image)
    context = ssl.create_default_context()
    print("context")

    try:
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.ehlo()
        server.starttls()
        server.ehlo()
        server.login(sender_email, omobot_pass)
        server.sendmail(sender_email, receiver_email, message.as_string())
        
        print("server.sendmail.......................")
        
        server.quit()
    except Exception as e:
        print(e)
    
    fallFileLoc = f"{mempath}fallInfo.txt"
    
    with lock:
        with open(fallFileLoc, 'w') as wf:
            wf.writelines(f"True, {time.time}")
    time.sleep(15)
    try:
        os.remove(fallFileLoc)
    except:
        pass
    
        





def falling_alarm(image, bbox):
    x_min, y_min, x_max, y_max = bbox
    cv2.rectangle(image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color=(0, 0, 255),
                  thickness=5, lineType=cv2.LINE_AA)
    cx, cy = int(x_min), int((y_min + y_max) / 2)
    cv2.putText(image, 'Person Fell down', (cx, cy), 0, 1, [
                0, 0, 2550], thickness=3, lineType=cv2.LINE_AA)

def prepare_image(image):
    _image = image[0].permute(1, 2, 0) * 255
    _image = _image.cpu().numpy().astype(np.uint8)
    _image = cv2.cvtColor(_image, cv2.COLOR_RGB2BGR)
    return _image

i = 0

model = YOLO('PoseDetection/yolov8n-pose.pt')
def Yolo8FallDetection(frame=None, ImSubs=None):
    global i,model, lock
    # load a pretrained model (recommended for training)
    # ImSubs.completedDetection =  False

    #try:
    #    frame = cv2.imread("testImage0.png")
    #except Exception as e:
    #    print(e)
    with lock:
        frame = cv2.imread(mempath + "testImage0.png")
        # np.load(data_location,mempath + "data.txt")
        print('yoloFall: Image is read.')
        if frame is None:
            print('yoloFall: Image is not ready.')
            return True
    print(type(frame))
    # try:
    if not type(frame) == None and type(frame) == np.ndarray:
        print('image is taken.')
        image = frame.copy()
        image = chng_perspective(image)
        print('image perspective changed.')
        # image = cv2.resize(image, (640, 480))
        shape = image.shape[:2]  # current shape [height, width]

        # Convert HWC to CHW, BGR to RGB
        image = image.transpose((2, 0, 1))[::-1]
        image = np.ascontiguousarray(image)
        image = torch.from_numpy(image)
        image = image.unsqueeze(dim=0)
        image = image / 255
        # Inference
        results = model(image, conf=0.3, show=False, device='cpu', save=False,  dnn =True)
        print('image is predicted.')
        image = prepare_image(image)

        fall_detected = False
        # for result in results:
        result = results[0]
        # print(result.)
        boxes = result.boxes  # Boxes object for bbox outputs
        for id, (cls, box, conf) in enumerate(zip(boxes.cls, boxes.xyxy, boxes.conf)):
            if cls == 0 and conf > 0.50:
                box = box.numpy()
                # Keypoints object for pose outputs
                kpt = result.keypoints.data[id].numpy()
                fall, _ = fall_detection(box, kpt)
                print('A person recognized. Fall:',
                    'True' if fall else 'False')
                # cv2.putText(image, str(conf.numpy()), (int(box[0]), int(
                #     box[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 150), 2)
                # for i, k in enumerate(kpt):
                #     x, y, c = k
                #     if c > 0.5:
                #         cv2.circle(image, (int(x), int(y)), 5, (255, 0, 0), -1)
                #         # cv2.putText(image, str(i), (int(x),int(y)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
                fall_detected = fall_detected or fall
                if fall:
                    falling_alarm(image, box)
        if fall_detected:
            cv2.imwrite('frame.png', image)
            print("Falled...........")
            sendEmail("Person1", 'frame.png')
            print("Email is sent...........")
        else:
            cv2.imwrite('frame.png', image)
        
        # Get the current time
        current_time = datetime.now()
        formatted_time = current_time.strftime("%Y-%m-%d %H:%M:%S")
        print("indx:", i, "Formatted time:", formatted_time)
        print("----------------------")
        i += 1
        # ImSubs.completedDetection = True
    # except Exception as e:
    #     print(e)
    
    return True


if __name__ == '__main__':
    global lock
    lock = FileLock(mempath + "file.lock")
    while True:
        time.sleep(0.1)
        ret = Yolo8FallDetection()
        if not ret:
            break
