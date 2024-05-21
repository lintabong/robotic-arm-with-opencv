import cv2
import time
import math
import serial
from ultralytics import YOLO

### CONFIGURATION
config = {
    'max_servo_angle': [160, 150, 180],
    'min_servo_angle': [10, 50, 50],
    'servo_angle': [90, 90, 150],
    'servo_step': [2, 2, 2],
    'depth': 0.5,
    'lenght_arm_1': 45,
    'lenght_arm_2': 45,
    'x_treshold': 5,
    'y_treshold': 5,
    'COM': 'COM4',
    'BAUDRATE': 115200,
    'video_port': 0,
    'state': 'scan', # berisi = scan, pickup_rubbish, send_trash_to_bin
    'trashes': ['remote', 'remote'], # masukkan sampah yang akan dibuang
    'scanning_state': 'left',
    'max_area_to_send_to_bin': 19932,
    'servo_angle_trash_to_bin': [90, 90, 168],
    'max_distance_sensor': 40,
    'motor_speed_at_case_1': [254, 250, 230, 250],
    'motor_speed_at_case_2': [254, 250, 254, 250],
    'motor_speed_at_case_3': [120, 120, 120, 120],
    'motor_speed_at_case_4': [120, 120, 120, 120],
    'motor_speed_at_forward': [120, 120, 120, 120],
    'motor_speed_at_backward': [254, 250, 254, 250],
    'motor_speed_at_left': [120, 120, 120, 120],
    'motor_speed_at_right': [120, 120, 120, 120],
    'motor_speed_at_stop': [0, 0, 0, 0],
    'robot_forward_direction' : [1, 1, 1, 1],
    'robot_backward_direction' : [0, 0, 0, 0],
    'robot_left_direction' : [1, 0, 1, 0],
    'robot_right_direction' : [0, 1, 0, 1],
    'pick': 1,
}

# melakukan inisiasi variabel yang akan digunakan
command = ''

ser = None
start_time = None

# serial connect
def connect_serial():
    ser = serial.Serial(config['COM'], config['BAUDRATE'])
    return ser

def robot_forward():
    command = str(config['servo_angle'][0])
    command += '.' + str(config['servo_angle'][1])
    command += '.' + str(config['servo_angle'][2])
    command += '.' + str(config['robot_forward_direction'][0])
    command += '.' + str(config['motor_speed_at_forward'][0])
    command += '.' + str(config['robot_forward_direction'][1])
    command += '.' + str(config['motor_speed_at_forward'][1])
    command += '.' + str(config['robot_forward_direction'][2])
    command += '.' + str(config['motor_speed_at_forward'][2])
    command += '.' + str(config['robot_forward_direction'][3])
    command += '.' + str(config['motor_speed_at_forward'][3])
    command += '.' + str(config['pick']) + '\n'

    ser.write(command.encode('utf-8'))

def robot_turn_right():
    command = str(config['servo_angle'][0])
    command += '.' + str(config['servo_angle'][1])
    command += '.' + str(config['servo_angle'][2])
    command += '.' + str(config['robot_right_direction'][0])
    command += '.' + str(config['motor_speed_at_right'][0])
    command += '.' + str(config['robot_right_direction'][1])
    command += '.' + str(config['motor_speed_at_right'][1])
    command += '.' + str(config['robot_right_direction'][2])
    command += '.' + str(config['motor_speed_at_right'][2])
    command += '.' + str(config['robot_right_direction'][3])
    command += '.' + str(config['motor_speed_at_right'][3])
    command += '.' + str(config['pick']) + '\n'

    ser.write(command.encode('utf-8'))

def robot_turn_left():
    command = str(config['servo_angle'][0])
    command += '.' + str(config['servo_angle'][1])
    command += '.' + str(config['servo_angle'][2])
    command += '.' + str(config['robot_left_direction'][0])
    command += '.' + str(config['motor_speed_at_left'][0])
    command += '.' + str(config['robot_left_direction'][1])
    command += '.' + str(config['motor_speed_at_left'][1])
    command += '.' + str(config['robot_left_direction'][2])
    command += '.' + str(config['motor_speed_at_left'][2])
    command += '.' + str(config['robot_left_direction'][3])
    command += '.' + str(config['motor_speed_at_left'][3])
    command += '.' + str(config['pick']) + '\n'

    ser.write(command.encode('utf-8'))

def robot_backward():
    command = str(config['servo_angle'][0])
    command += '.' + str(config['servo_angle'][1])
    command += '.' + str(config['servo_angle'][2])
    command += '.' + str(config['robot_backward_direction'][0])
    command += '.' + str(config['motor_speed_at_backward'][0])
    command += '.' + str(config['robot_backward_direction'][1])
    command += '.' + str(config['motor_speed_at_backward'][1])
    command += '.' + str(config['robot_backward_direction'][2])
    command += '.' + str(config['motor_speed_at_backward'][2])
    command += '.' + str(config['robot_backward_direction'][3])
    command += '.' + str(config['motor_speed_at_backward'][3])
    command += '.' + str(config['pick']) + '\n'

    ser.write(command.encode('utf-8'))

def robot_stop():
    command = str(config['servo_angle'][0])
    command += '.' + str(config['servo_angle'][1])
    command += '.' + str(config['servo_angle'][2])
    command += '.' + str(config['robot_forward_direction'][0])
    command += '.' + str(config['motor_speed_at_stop'][0])
    command += '.' + str(config['robot_forward_direction'][1])
    command += '.' + str(config['motor_speed_at_stop'][1])
    command += '.' + str(config['robot_forward_direction'][2])
    command += '.' + str(config['motor_speed_at_stop'][2])
    command += '.' + str(config['robot_forward_direction'][3])
    command += '.' + str(config['motor_speed_at_stop'][3])
    command += '.' + str(config['pick']) + '\n'

    ser.write(command.encode('utf-8'))

def move_servo_v2(x, y, z):
    b = math.atan2(y, x) * (180 / math.pi)
    l = math.sqrt(x*x + y*y)
    h = math.sqrt(l*l + z*z)
    phi = math.atan(z / l) * (180 / math.pi)
    theta = math.acos(h / (2 * 75)) * (180 / math.pi)
    
    a1 = phi + theta
    a2 = phi - theta

    command = str(b)
    command += '.' + str(a1)
    command += '.' + str(a2)
    command += '.' + str(config['robot_forward_direction'][0])
    command += '.' + str(config['motor_speed_at_case_1'][0])
    command += '.' + str(config['robot_forward_direction'][1])
    command += '.' + str(config['motor_speed_at_case_1'][1])
    command += '.' + str(config['robot_forward_direction'][2])
    command += '.' + str(config['motor_speed_at_case_1'][2])
    command += '.' + str(config['robot_forward_direction'][3])
    command += '.' + str(config['motor_speed_at_case_1'][3])
    command += '.' + str(config['pick']) + '\n'

    ser.write(command.encode('utf-8'))

def servo_move_to_axes(x, y, l1=config['lenght_arm_1'], l2=config['lenght_arm_1']):
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

    theta1_a_deg = math.degrees(theta1_a)
    theta2_a_deg = math.degrees(theta2_a)
    theta1_b_deg = math.degrees(theta1_b)
    theta2_b_deg = math.degrees(theta2_b)

    theta1_a_deg = abs(int(theta1_a_deg))
    theta2_a_deg = abs(abs(int(180 - theta2_a_deg))) + 60
    
    theta1_b_deg = abs(int(180 - theta1_b_deg))
    theta2_b_deg = abs(abs(int(180 - theta2_b_deg))) + 60
    
    # return (theta1_a_deg, theta2_a_deg), (theta1_b_deg, theta2_b_deg)

    command = str(config['servo_angle'][0])
    command += '.' + str(theta1_b_deg)
    command += '.' + str(theta2_a_deg)
    command += '.' + str(config['robot_forward_direction'][0])
    command += '.' + str(config['motor_speed_at_case_1'][0])
    command += '.' + str(config['robot_forward_direction'][1])
    command += '.' + str(config['motor_speed_at_case_1'][1])
    command += '.' + str(config['robot_forward_direction'][2])
    command += '.' + str(config['motor_speed_at_case_1'][2])
    command += '.' + str(config['robot_forward_direction'][3])
    command += '.' + str(config['motor_speed_at_case_1'][3])
    command += '.' + str(config['pick']) + '\n'

    ser.write(command.encode('utf-8'))

if __name__ == '__main__':
    ser = connect_serial()

    time.sleep(2)

    # servo_move_to_axes(60, 20)
    # time.sleep(2)
    # servo_move_to_axes(50, 20)

    start_time = time.time()

    robot_turn_left()

    time.sleep(1)
    
    robot_forward()

    time.sleep(2)

    robot_backward()

    time.sleep(2)

    robot_stop()
