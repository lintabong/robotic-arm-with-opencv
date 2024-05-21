import cv2
import time
import math
import serial
from ultralytics import YOLO

### CONFIGURATION
config = {
    'max_servo_angle': [160, 150, 180, 150, 150],
    'min_servo_angle': [10, 50, 50, 50, 50],
    'servo_angle': [90, 90, 168, 90, 90],
    'servo_step': [2, 2, 2, 2, 2],
    'depth': 0.5,
    'x_treshold': 5,
    'y_treshold': 5,
    'COM': 'COM8',
    'BAUDRATE': 115200,
    'video_port': 0,
    'state': 'scan', # berisi = scan, pickup_rubbish, send_trash_to_bin
    'trashes': ['remote', 'remote'], # masukkan sampah yang akan dibuang
    'scanning_state': 'left',
    'max_area_to_send_to_bin': 19932,
    'servo_angle_trash_to_bin': [90, 90, 168, 90, 90],
    'max_distance_sensor': 40
}

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

def scale_value(old_value, old_min, old_max, new_min, new_max):
    return int((new_max - new_min) / (old_max - old_min) * (old_value - old_min) + new_min)

def calculate_servo_angles(x, y, l1=10, l2=10):
    theta1 = math.degrees(math.atan2(y, x))

    r = max(math.sqrt(x**2 + y**2) - l1, 0)

    a = min((l1**2 + l2**2 - r**2) / (2 * l1 * l2), 1)
    theta2 = math.degrees(math.acos(a) if a >= 0 else math.acos(-1))
    return theta1, theta2

def send_command(ser, servo_angle):
    command = '.'.join(map(str, servo_angle))
    ser.write((command + '\n').encode('utf-8'))

def main():
    ser = serial.Serial(config['COM'], config['BAUDRATE'])

    cap = cv2.VideoCapture(config['video_port'])
    cap.set(3, 640)
    cap.set(4, 480)
    model = YOLO("yolo-Weights/yolov8n.pt", verbose=False)
    # model = YOLO("best.pt", verbose=False)

    start_servo_angle = config['servo_angle']

    time.sleep(2)

    send_command(ser, config['servo_angle'])

    while True:
        _, img = cap.read()
        results = model(img, stream=True, verbose=False)

        # logic untuk scan sampah
        if config['state'] == 'scan_by_wheel':
            ser.write("22.22.22.22.22.right.124\n".encode())

            time.sleep(2)

        if config['state'] == 'scan':
            if config['scanning_state'] == 'left':
                config['servo_angle'][0] += config['servo_step'][0]

                if config['servo_angle'][0] >= config['max_servo_angle'][0]:
                    config['scanning_state'] = 'right'
            
            else:
                config['servo_angle'][0] -= config['servo_step'][0]

                if config['servo_angle'][0] <= config['min_servo_angle'][0]:
                    config['scanning_state'] = 'left'

            config['servo_step'][0] = 1

            send_command(ser, config['servo_angle'])

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
                    cv2.putText(img, classNames[cls], org, font, fontScale, color, thickness)

                    if classNames[cls] in config['trashes']:
                        print(f'found {classNames[cls]}')

                        config['state'] = 'pickup_rubbish'

        # logic jika mendapatkan sampah
        if config['state'] == 'pickup_rubbish':
            config['servo_step'][0]  = 2

            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 255), 3)

                    cls = int(box.cls[0])
                    if classNames[cls] in config['trashes']:
                        x_mid, y_mid = (x1 + x2) // 2, (y1 + y2) // 2
                        width, height = x2 - x1, y2 - y1

                        if x_mid - 320 >= config['x_treshold']:
                            config['servo_angle'][0] -= config['servo_step'][0]
                        if x_mid - 320 <= config['x_treshold']:
                            config['servo_angle'][0] += config['servo_step'][0]
                        if y_mid - 240 >= config['y_treshold']:
                            config['servo_angle'][2] += config['servo_step'][2]
                        if y_mid - 240 <= config['y_treshold']:
                            config['servo_angle'][2] -= config['servo_step'][2]

                        for i in range(5):
                            config['servo_angle'][i] = max(config['min_servo_angle'][i], min(config['max_servo_angle'][i], config['servo_angle'][i]))

                        send_command(ser, config['servo_angle'])

                        if width*height >= config['max_area_to_send_to_bin']:
                            print('sending trash to bin')

                            # config['state'] = 'send_trash_to_bin'

                        print(classNames[cls], config['servo_angle'], (x_mid, y_mid), width, height, width*height)

        if config['state'] == 'send_trash_to_bin':
            config['servo_angle'][1] = 140
            send_command(ser, config['servo_angle'])

            config['trashes'].remove(classNames[cls])

            config['state'] = 'scan'

        # proses logging di frame
        text = config['state'] 
        org = (20, 20)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        color = (255, 0, 0)
        thickness = 2
        cv2.putText(img, text, org, font, fontScale, color, thickness)

        # jika sampah sudah habis, maka end proses
        if len(config['trashes']) <= 0:
            break

        # escape dari proses
        cv2.imshow('Webcam', img)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    time.sleep(2)

    send_command(ser, start_servo_angle)

if __name__ == "__main__":
    main()
