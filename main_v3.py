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
    'state': 'scan',
    'trashes': ['remote', 'remote'],
    'scanning_state': 'left',
    'max_area_to_send_to_bin': 19932,
    'servo_angle_trash_to_bin': [90, 90, 168],
    'max_distance_sensor': 40,
    'connect_serial': False,
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
start_time = None

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

def move_servo_v2(x, y, z):
    b = math.degrees(math.atan2(y, x))
    l = math.sqrt(x*x + y*y)
    h = math.sqrt(l*l + z*z)
    phi = math.degrees(math.atan(z / l))
    theta = math.degrees(math.acos(h / (2 * 75)))
    a1, a2 = phi + theta, phi - theta

    command = f"{b}.{a1}.{a2}." + '.'.join(map(str, config['robot_direction']['forward'] + config['motor_speed']['forward'] + [config['pick']])) + '\n'
    ser.write(command.encode('utf-8'))

def servo_move_to_axes(x, y, l1=config['lenght_arm_1'], l2=config['lenght_arm_2']):
    r = math.sqrt(x**2 + y**2)
    if r > (l1 + l2) or r < abs(l1 - l2):
        return None, None

    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.acos(cos_theta2)
    k1, k2 = l1 + l2 * math.cos(theta2), l2 * math.sin(theta2)
    theta1 = math.degrees(math.atan2(y, x) - math.atan2(k2, k1))
    theta2_deg = abs(abs(int(180 - math.degrees(theta2)))) + 60

    command = f"{config['servo_angle'][0]}.{abs(int(180 - theta1))}.{theta2_deg}." + '.'.join(map(str, config['robot_direction']['forward'] + config['motor_speed']['forward'] + [config['pick']])) + '\n'
    ser.write(command.encode('utf-8'))

if __name__ == '__main__':
    if config['connect_serial']:
        ser = connect_serial()
        time.sleep(2)
    print("start")

    command = robot_turn_left()
    if config['connect_serial']:
        send_command(command)
    print('left ', command)
    time.sleep(1)

    command = robot_forward()
    if config['connect_serial']:
        send_command(command)
    print('forward', command)
    time.sleep(2)

    command = robot_backward()
    if config['connect_serial']:
        send_command(command)
    print('backward', command)
    time.sleep(2)

    command = robot_stop()
    if config['connect_serial']:
        send_command(command)
    print('stop', command)
