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
    'depth': 0.5,
    'lenght_arm_1': 45,
    'lenght_arm_2': 45,
    'x_treshold': 5,
    'y_treshold': 5,
    'x_cam_on_scanning': 20,
    'y_cam_on_scanning': 55,
    'z_cam_on_scanning': 0,
    'COM': 'COM4',
    'BAUDRATE': 115200,
    'video_port': 0,
    'trashes': ['remote'],
    'table_approaching_time': 1, # per 0.05 second
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
history_state = []

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
    global ser
    if config['connect_serial']:
        ser = serial.Serial(config['COM'], config['BAUDRATE'])

def build_command(direction, speed):
    return f"{config['servo_angle'][0]}.{config['servo_angle'][1]}.{config['servo_angle'][2]}." \
           f"{config['robot_direction'][direction][0]}.{config['robot_speed'][speed][0]}." \
           f"{config['robot_direction'][direction][1]}.{config['robot_speed'][speed][1]}." \
           f"{config['robot_direction'][direction][2]}.{config['robot_speed'][speed][2]}." \
           f"{config['robot_direction'][direction][3]}.{config['robot_speed'][speed][3]}." \
           f"{config['pick']}\n"

def send_command(command):
    if config['connect_serial']:
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
        return 110, 125

    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)

    theta2_a = math.acos(cos_theta2)
    theta2_b = -math.acos(cos_theta2)
    
    k1_a = l1 + l2 * math.cos(theta2_a)
    k2_a = l2 * math.sin(theta2_a)

    k1_b = l1 + l2 * math.cos(theta2_b)
    k2_b = l2 * math.sin(theta2_b)
    theta1_b = math.atan2(y, x) - math.atan2(k2_b, k1_b)

    theta2_a_deg = math.degrees(theta2_a)
    theta1_b_deg = math.degrees(theta1_b)

    theta1_b_deg = abs(int(180 - theta1_b_deg))
    theta2_a_deg = abs(int(180 - theta2_a_deg)) + 60

    # return command
    return f"{config['servo_angle'][0]}.{theta1_b_deg}.{theta2_a_deg}." \
           f"{config['robot_direction']['forward'][0]}.{config['robot_speed']['stop'][0]}." \
           f"{config['robot_direction']['forward'][1]}.{config['robot_speed']['stop'][1]}." \
           f"{config['robot_direction']['forward'][2]}.{config['robot_speed']['stop'][2]}." \
           f"{config['robot_direction']['forward'][3]}.{config['robot_speed']['stop'][3]}." \
           f"{config['pick']}\n"

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

def logging(command, his=True):
    global history
    command = command.replace('\n', '')
    print('    ', command)
    if his:
        history.append(command)


def test_servo_square():
    servo_angle_1 = config['servo_angle'][1]
    servo_angle_2 = config['servo_angle'][2]

    x = 20
    y = 4

    command = servo_move_to_axes(x, y)
    
    last_servo_angle = command.split('.')
    last_servo_angle_1 = int(last_servo_angle[1])
    last_servo_angle_2 = int(last_servo_angle[2])

    print('initial condition', servo_angle_1, last_servo_angle_1, servo_angle_2, last_servo_angle_2)

    while True:
        if servo_angle_1 < last_servo_angle_1:
            servo_angle_1 += 1

        if servo_angle_1 > last_servo_angle_1:
            servo_angle_1 -= 1

        if servo_angle_2 < last_servo_angle_2:
            servo_angle_2 += 1

        if servo_angle_2 > last_servo_angle_2:
            servo_angle_2 -= 1

        command = servo_move_by_angle(servo_angle_1, servo_angle_2)
        send_command(command)
        logging(command, False)

        time.sleep(0.3)

        if servo_angle_1 == last_servo_angle_1 and servo_angle_2 == last_servo_angle_2:
            break
    y = 4
    for x in range(20, 60):
        command = servo_move_to_axes(x, y)
        send_command(command)

        print((x, y), command.replace('\n', ''))
        time.sleep(1)

    for y in range(4, 55):
        command = servo_move_to_axes(x, y)
        send_command(command)

        print((x, y), command.replace('\n', ''))
        time.sleep(1)

        
    for x in range(60, 20, -1):
        command = servo_move_to_axes(x, y)
        send_command(command)

        print((x, y), command.replace('\n', ''))
        time.sleep(1)

    for y in range(55, 4, -1):
        command = servo_move_to_axes(x, y)
        send_command(command)

        print((x, y), command.replace('\n', ''))
        time.sleep(1)

if __name__ == '__main__':
    # LANGKAH KE-1
    # membuka koneksi ke serial
    connect_serial()
    time.sleep(2)

    # LANGKAH KE-2
    # robot maju mendekati meja
    for _ in range(int(config['table_approaching_time'])):
        command = robot_turn_left()
        send_command(command)
        logging(command)

        time.sleep(0.05)

    # setelah robot bergerak ke kiri dengan waktu dan kecepatan tertentu
    # robot akan berhenti
    command = robot_stop()
    send_command(command)
    logging(command)

    # test servo square
    # print('test servo square')
    # test_servo_square()

    # LANGKAH KE-3
    # setting sudut awal ke posisi scan dengan iterasi sebesar 1 derajat
    servo_angle_1 = config['servo_angle'][1]
    servo_angle_2 = config['servo_angle'][2]

    x_cam = config['x_cam_on_scanning']
    y_cam = config['y_cam_on_scanning']
    z_cam = config['z_cam_on_scanning']

    command = servo_move_to_axes(x_cam, y_cam)
    
    last_servo_angle = command.split('.')
    last_servo_angle_1 = int(last_servo_angle[1])
    last_servo_angle_2 = int(last_servo_angle[2])

    while True:
        if servo_angle_1 <= last_servo_angle_1:
            servo_angle_1 += 1

        if servo_angle_1 >= last_servo_angle_1:
            servo_angle_1 -= 1

        if servo_angle_2 <= last_servo_angle_2:
            servo_angle_2 += 1

        if servo_angle_2 >= last_servo_angle_2:
            servo_angle_2 -= 1

        command = servo_move_by_angle(servo_angle_1, servo_angle_2)
        send_command(command)

        time.sleep(0.3)

        if servo_angle_1 == last_servo_angle_1 and servo_angle_2 == last_servo_angle_2:
            break

    # LANGKAH KE-4
    # membuka koneksi kamera
    connect_camera()
    time.sleep(2)

    # LANGKAH KE-5
    # logic membuang sampah
    # logic dilakukan dengan cara melakukan iterasi terhadap sampah yang dimasukkan ke config['trashes']
    # untuk tiap sampah, dilakukan proses scan, centering trash, pick up trash, throw trash
    for i, trash in enumerate(config['trashes']):
        history = []
        print('SAMPAH ke-', i+1, trash)

        command = robot_stop()
        send_command(command)
        logging(command, False)

        state = 'scan'
        area = 0
        forward = 0
        backward = 0

        command = servo_move_to_axes(x_cam, z_cam)
        send_command(command)
        logging(command, False)

        # proses scanning sampah
        print('    scanning')
        while True:
            break
