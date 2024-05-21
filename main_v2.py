import cv2
import time
import math
import serial
from ultralytics import YOLO

### CONFIGURATION
config = {
    'max_servo_angle': [160, 150, 180],
    'min_servo_angle': [10, 50, 50],
    'servo_angle': [90, 90, 168],
    'servo_step': [2, 2, 2],
    'depth': 0.5,
    'lenght_arm_1': 20,
    'lenght_arm_2': 20,
    'x_treshold': 5,
    'y_treshold': 5,
    'COM': 'COM7',
    'BAUDRATE': 115200,
    'video_port': 0,
    'state': 'scan', # berisi = scan, pickup_rubbish, send_trash_to_bin
    'trashes': ['remote', 'remote'], # masukkan sampah yang akan dibuang
    'scanning_state': 'left',
    'max_area_to_send_to_bin': 19932,
    'servo_angle_trash_to_bin': [90, 90, 168],
    'max_distance_sensor': 40,
    'motor_speed_at_case_1': [120, 120, 120, 120],
    'motor_speed_at_case_2': [120, 120, 120, 120],
    'motor_speed_at_case_3': [120, 120, 120, 120],
    'motor_speed_at_stop': [0, 0, 0, 0],
    'robot_forward_direction' : [1, 1, 1, 1],
    'robot_backward_direction' : [1, 1, 1, 1],
    'robot_left_direction' : [1, 1, 1, 1],
    'robot_right_direction' : [1, 1, 1, 1],
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

# robot turn left
def robot_forward():
    command = str(config['servo_angle'][0])
    command += '.' + str(config['servo_angle'][1])
    command += '.' + str(config['servo_angle'][2])
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

# robot turn right
def robot_turn_right():
    command = str(config['servo_angle'][0])
    command += '.' + str(config['servo_angle'][1])
    command += '.' + str(config['servo_angle'][2])
    command += '.' + str(config['robot_right_direction'][0])
    command += '.' + str(config['motor_speed_at_case_2'][0])
    command += '.' + str(config['robot_right_direction'][1])
    command += '.' + str(config['motor_speed_at_case_2'][1])
    command += '.' + str(config['robot_right_direction'][2])
    command += '.' + str(config['motor_speed_at_case_2'][2])
    command += '.' + str(config['robot_right_direction'][3])
    command += '.' + str(config['motor_speed_at_case_2'][3])
    command += '.' + str(config['pick']) + '\n'

    ser.write(command.encode('utf-8'))

# robot turn left
def robot_turn_left():
    command = str(config['servo_angle'][0])
    command += '.' + str(config['servo_angle'][1])
    command += '.' + str(config['servo_angle'][2])
    command += '.' + str(config['robot_left_direction'][0])
    command += '.' + str(config['motor_speed_at_case_2'][0])
    command += '.' + str(config['robot_left_direction'][1])
    command += '.' + str(config['motor_speed_at_case_2'][1])
    command += '.' + str(config['robot_left_direction'][2])
    command += '.' + str(config['motor_speed_at_case_2'][2])
    command += '.' + str(config['robot_left_direction'][3])
    command += '.' + str(config['motor_speed_at_case_2'][3])
    command += '.' + str(config['pick']) + '\n'

    ser.write(command.encode('utf-8'))

# robot stop
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

    

    # move_to_angle(b, a1, a2)

def move_servo(x, y, z=None, last_x=None, last_y=None):
    cos_theta2 = (x**2 + y**2 - config['lenght_arm_1']**2 - config['lenght_arm_2']**2) / (2 * config['lenght_arm_1'] * config['lenght_arm_2'])
    
    if -1 <= cos_theta2 <= 1:
        theta2 = math.acos(cos_theta2)
    else:
        theta2 = 80
    
    k1 = config['lenght_arm_1'] + config['lenght_arm_2'] * math.cos(theta2)
    k2 = config['lenght_arm_1'] * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)
    
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)

    if (theta1_deg >= config['max_servo_angle'][1]):
        theta1_deg == config['max_servo_angle'][1]

    if (theta1_deg <= config['min_servo_angle'][1]):
        theta1_deg == config['min_servo_angle'][1]

    if (theta2_deg >= config['max_servo_angle'][2]):
        theta2_deg == config['max_servo_angle'][2]

    if (theta2_deg >= config['min_servo_angle'][2]):
        theta2_deg == config['min_servo_angle'][2]

    command = str(config['servo_angle'][0])
    command += '.' + str(theta1_deg)
    command += '.' + str(theta2_deg)
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
    
    return theta1_deg, theta2_deg

if __name__ == '__main__':
    ser = connect_serial()

    time.sleep(2)

    start_time = time.time()

    robot_forward()

    time.sleep(1)
    
    robot_turn_left()

    time.sleep(1)

    robot_turn_right()

    time.sleep(1)

    robot_stop()

    time.sleep(1)
