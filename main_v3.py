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
    'lenght_arm_1': 50,
    'lenght_arm_2': 50,
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
    'motor_speed_at_case_2': [254, 254, 250, 250],
    'motor_speed_at_case_3': [120, 120, 120, 120],
    'motor_speed_at_case_4': [120, 120, 120, 120],
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
    command += '.' + str(config['motor_speed_at_case_3'][0])
    command += '.' + str(config['robot_left_direction'][1])
    command += '.' + str(config['motor_speed_at_case_3'][1])
    command += '.' + str(config['robot_left_direction'][2])
    command += '.' + str(config['motor_speed_at_case_3'][2])
    command += '.' + str(config['robot_left_direction'][3])
    command += '.' + str(config['motor_speed_at_case_3'][3])
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

def move_servo(x, y, z=None, a1=50, a2=50):
    cos_q2 = (x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2)
    cos_q2 = max(min(cos_q2, 1), -1)
    q2 = math.acos(cos_q2)

    q1 = math.atan2(y, x) - math.atan2(a2 * math.sin(q2), a1 + a2 * math.cos(q2))

    q1_deg = math.degrees(q1)
    q2_deg = math.degrees(q2)


    if (q1_deg >= config['max_servo_angle'][1]):
        q1_deg == config['max_servo_angle'][1]

    if (q1_deg <= config['min_servo_angle'][1]):
        q1_deg == config['min_servo_angle'][1]

    if (q2_deg >= config['max_servo_angle'][2]):
        q2_deg == config['max_servo_angle'][2]

    if (q2_deg >= config['min_servo_angle'][2]):
        q2_deg == config['min_servo_angle'][2]

    command = str(config['servo_angle'][0])
    command += '.' + str(q1_deg)
    command += '.' + str(q2_deg)
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
    
    return q1_deg, q2_deg

if __name__ == '__main__':
    ser = connect_serial()

    time.sleep(2)

    start_time = time.time()

    robot_forward()

    time.sleep(4)
    
    robot_turn_left()

    time.sleep(4)

    robot_turn_right()

    time.sleep(4)

    robot_stop()

    time.sleep(4)

    move_servo(x=40, y=50, z=90)
