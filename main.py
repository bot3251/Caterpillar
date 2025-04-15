import threading
import serial
import time
import cv2
import queue

# Очередь команд
command_queue = queue.Queue()
is_marker=False;
# Поток, который слушает очередь и отправляет в serial
class SerialSender(threading.Thread):
    def __init__(self, port='COM8', baudrate=9600):
        super().__init__()
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        self.running = True

    def run(self):
        while self.running:
            try:
                value = command_queue.get(timeout=1)
                self.ser.write(str(value).encode())
                print(f"[Serial] Отправлено: {value}")
            except queue.Empty:
                continue

    def stop(self):
        self.running = False
        self.ser.close()

    def read(self):
        return str(self.ser.readline().decode("utf-8")[:-2])

# Запускаем поток один раз
serial_thread = SerialSender()

serial_thread.daemon = True
serial_thread.start()

def start():
    global is_marker
    k = -1
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    params = cv2.aruco.DetectorParameters()
    print("Начали")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Ошибка: Не удалось открыть камеру.")
        exit()

    while k == -1:
        ret, img = cap.read()
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = aruco_detector.detectMarkers(img)

        if ids is not None:
            img = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)

            # Добавляем команду в очередь
            if not(is_marker):
                command_queue.put(ids[0][0])  # например, отправляем первый ID
                is_marker=True
        else:
            print("Маркер не обнаружен.")
            if is_marker:
                command_queue.put(-1)
                is_marker=False
        cv2.imshow("Detected ArUco Markers", img)
        k = cv2.waitKey(10)
    # cap.release()
    # cv2.destroyAllWindows()
    serial_thread.stop()  # останавливаем поток

while True:
    data = serial_thread.read()
    if data=="start":
        start()
        serial_thread.stop()
        print("hi")