import cv2
import time
import math
import serial
from ultralytics import YOLO

### CONFIGURATION
config = {
    'max_servo_angle': [90, 140, 150],
    'min_servo_angle': [90, 80, 100],
    'servo_angle': [90, 90, 150],
    'servo_step': [2, 2, 2],
    'servo_angle_trash_to_bin': [90, 90, 168],
    'depth': 0.5,
    'lenght_arm_1': 45,
    'lenght_arm_2': 45,
    'x_treshold': 5,
    'y_treshold': 5,
    'COM': 'COM4',
    'BAUDRATE': 115200,
    'video_port': 0,
    'trashes': ['remote'],
    'max_area_to_send_to_bin': 19932,
    'max_distance_sensor': 40,
    'connect_serial': True,
    'robot_speed': {
        'forward': [120, 120, 120, 120],
        'backward': [254, 250, 254, 250],
        'left': [120, 120, 120, 120],
        'right': [120, 120, 120, 120],
        'stop': [0, 0, 0, 0],
        'case_1': [0, 0, 0, 0],
    },
    'robot_direction': {
        'forward': [1, 1, 1, 1],
        'backward': [0, 0, 0, 0],
        'left': [1, 0, 1, 0],
        'right': [0, 1, 0, 1]
    },
    'pick': 1,
}

# melakukan inisiasi variabel yang akan digunakan (tidak perlu dirubah)
command = ''
ser = None
cap = None
model = YOLO('yolo-Weights/yolov8n.pt', verbose=False)
start_time = None
history = []

classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]

# classNames = ["Botol", "Daun Fresh", "Daun Kering", "Ferro", "Kertas Bungkus", 
#               "Koran", "Non-Ferro", "Plastik Ijo", "Plastik Putih"]

def connect_camera():
    global model, cap
    cap = cv2.VideoCapture(config['video_port'])
    cap.set(3, 640)
    cap.set(4, 480)

def connect_serial():
    return serial.Serial(config['COM'], config['BAUDRATE'])

def build_command(direction, speed):
    command = str(config['servo_angle'][0])
    command += '.' + str(config['servo_angle'][1])
    command += '.' + str(config['servo_angle'][2])
    command += '.' + str(config['robot_direction'][direction][0])
    command += '.' + str(config['robot_speed'][speed][0])
    command += '.' + str(config['robot_direction'][direction][1])
    command += '.' + str(config['robot_speed'][speed][1])
    command += '.' + str(config['robot_direction'][direction][2])
    command += '.' + str(config['robot_speed'][speed][2])
    command += '.' + str(config['robot_direction'][direction][3])
    command += '.' + str(config['robot_speed'][speed][3])
    command += '.' + str(config['pick']) + '\n'
    
    return command

def send_command(command):
    ser.write(command.encode('utf-8'))

def robot_forward():
    return build_command('forward', 'forward')

def robot_backward():
    return build_command('backward', 'backward')

def robot_turn_left():
    return build_command('left', 'left')

def robot_turn_right():
    return build_command('right', 'right')

def robot_stop():
    return build_command('forward', 'stop')

def servo_move_to_axes(x, y, l1=config['lenght_arm_1'], l2=config['lenght_arm_2']):
    r = math.sqrt(x**2 + y**2)

    if r > (l1 + l2) or r < abs(l1 - l2):
        return None, None

    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    theta2_a = math.acos(cos_theta2)
    theta2_b = -math.acos(cos_theta2)
    
    k1_a = l1 + l2 * math.cos(theta2_a)
    k2_a = l2 * math.sin(theta2_a)
    theta1_a = math.atan2(y, x) - math.atan2(k2_a, k1_a)
    
    k1_b = l1 + l2 * math.cos(theta2_b)
    k2_b = l2 * math.sin(theta2_b)
    theta1_b = math.atan2(y, x) - math.atan2(k2_b, k1_b)

    theta2_a_deg = math.degrees(theta2_a)
    theta1_b_deg = math.degrees(theta1_b)

    theta1_b_deg = abs(int(180 - theta1_b_deg))

    theta2_a_deg = abs(int(180 - theta2_a_deg)) + 60

    # if theta1_b_deg >= config['max_servo_angle'][1]:
    #     theta1_b_deg = config['max_servo_angle'][1]

    # if theta1_b_deg <= config['min_servo_angle'][1]:
    #     theta1_b_deg = config['min_servo_angle'][1]

    # if theta2_a_deg >= config['max_servo_angle'][2]:
    #     theta2_a_deg = config['max_servo_angle'][2]

    # if theta2_a_deg <= config['min_servo_angle'][2]:
    #     theta2_a_deg = config['min_servo_angle'][2]

    command = str(config['servo_angle'][0])
    command += '.' + str(theta1_b_deg)
    command += '.' + str(theta2_a_deg)
    command += '.' + str(config['robot_direction']['forward'][0])
    command += '.' + str(config['robot_speed']['stop'][0])
    command += '.' + str(config['robot_direction']['forward'][1])
    command += '.' + str(config['robot_speed']['stop'][1])
    command += '.' + str(config['robot_direction']['forward'][2])
    command += '.' + str(config['robot_speed']['stop'][2])
    command += '.' + str(config['robot_direction']['forward'][3])
    command += '.' + str(config['robot_speed']['stop'][3])
    command += '.' + str(config['pick']) + '\n'
    return command


def servo_move_by_angle(tetha1, tetha2):
    command = str(config['servo_angle'][0])
    command += '.' + str(tetha1)
    command += '.' + str(tetha2)
    command += '.' + str(config['robot_direction']['forward'][0])
    command += '.' + str(config['robot_speed']['stop'][0])
    command += '.' + str(config['robot_direction']['forward'][1])
    command += '.' + str(config['robot_speed']['stop'][1])
    command += '.' + str(config['robot_direction']['forward'][2])
    command += '.' + str(config['robot_speed']['stop'][2])
    command += '.' + str(config['robot_direction']['forward'][3])
    command += '.' + str(config['robot_speed']['stop'][3])
    command += '.' + str(config['pick']) + '\n'
    
    return command

def test_servo_square():
    for x in range(30, 65):
        command = servo_move_to_axes(x, 20)
        if config['connect_serial']:
            send_command(command)

        print(command)
        time.sleep(0.5)

    for y in range(15, 45):
        command = servo_move_to_axes(55, y)
        if config['connect_serial']:
            send_command(command)

        print(command)
        time.sleep(0.5)

        
    for x in range(65, 30, -1):
        command = servo_move_to_axes(x, 40)
        if config['connect_serial']:
            send_command(command)

        print(command)
        time.sleep(0.5)

    for y in range(45, 15, -1):
        command = servo_move_to_axes(40, y)
        if config['connect_serial']:
            send_command(command)

        print(command)
        time.sleep(0.5)

if __name__ == '__main__':
    print('start')

    # membuka koneksi ke serial
    if config['connect_serial']:
        ser = connect_serial()
        time.sleep(2)
    

    # robot maju mendekati meja
    for i in range(20):
        command = robot_turn_left()
        if config['connect_serial']:
            send_command(command)

        command = command.replace('\n', '')
        history.append(command)
        print(command)

        time.sleep(0.05)

    command = robot_stop()
    if config['connect_serial']:
        send_command(command)
    
    command = command.replace('\n', '')
    history.append(command)
    print(command)

    # membuka koneksi ke kamera
    connect_camera()
    print('connect camera')
    time.sleep(1)

    # clear history
    history = []

    for trash in config['trashes']:
        command = robot_stop()
        if config['connect_serial']:
            send_command(command)

        command = command.replace('\n', '')
        history.append(command)
        print(command)

        state = 'scan'
        area = 0
        forward = 0
        backward = 0

        x_cam = 20
        z_cam = 40

        command = servo_move_to_axes(x_cam, z_cam)
        if config['connect_serial']:
            send_command(command)

        command = command.replace('\n', '')
        history.append(command)
        print(command)

        while True:
            if state == 'scan':
                if forward >= 1000:
                    command = robot_backward()
                    if config['connect_serial']:
                        send_command(command)
                else:
                    command = robot_forward()
                    if config['connect_serial']:
                        send_command(command)

                    forward += 1

                _, img = cap.read()
                results = model(img, stream=True, verbose=False)

                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])

                        cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                        org = [x1, y1]
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        fontScale = 1
                        color = (255, 0, 0)
                        thickness = 2

                        cls = int(box.cls[0])

                    if classNames[cls] == trash:
                        command = robot_stop()
                        if config['connect_serial']:
                            send_command(command)

                        state = 'centering_trash'

                command = command.replace('\n', '')
                history.append(command)
                print(command)

            if state == 'centering_trash':
                command = robot_stop()
                if config['connect_serial']:
                    send_command(command)

                command = command.replace('\n', '')
                history.append(command)
                print(command)

                _, img = cap.read()
                results = model(img, stream=True, verbose=False)

                for r in results:
                    for box in r.boxes:
                        x1, z1, x2, z2 = map(int, box.xyxy[0])

                        cv2.rectangle(img, (x1, z1), (x2, z2), (255, 0, 255), 3)

                        org = [x1, z1]
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        fontScale = 1
                        color = (255, 0, 0)
                        thickness = 2

                        cls = int(box.cls[0])
                        cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

                        if classNames[cls] == trash:
                            x_mid, z_mid = (x1 + x2) // 2, (z1 + z2) // 2
                            width, height = x2 - x1, z2 - z1

                            print(x_mid, z_mid, width, height)

                            if x_mid - 320 >= config['x_treshold']:
                                x_cam -= 2

                            if x_mid - 320 <= config['x_treshold']:
                                x_cam += 2

                            if z_mid - 240 >= config['y_treshold']:
                                command = robot_forward()
                                if config['connect_serial']:
                                    send_command(command)

                                command = command.replace('\n', '')
                                history.append(command)
                                print(command)

                            if z_mid - 240 <= config['y_treshold']:
                                command = robot_backward()
                                if config['connect_serial']:
                                    send_command(command)

                                command = command.replace('\n', '')
                                history.append(command)
                                print(command)

                            command = servo_move_to_axes(x_cam, z_cam)
                            if config['connect_serial']:
                                send_command(command)

                            command = command.replace('\n', '')
                            history.append(command)
                            print(command)

            if state == 'pick_up_trash':
                pass

            if state == 'throw_trash':
                command = servo_move_to_axes()
                if config['connect_serial']:
                    send_command(command)

                command = command.replace('\n', '')
                history.append(command)
                print(command)

                break

            # escape dari proses
            cv2.imshow('Webcam', img)
            if cv2.waitKey(1) == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

    time.sleep(2)
