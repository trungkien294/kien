import cv2
import pytesseract
import pyodbc
import re
import serial
import time
from ultralytics import YOLO

# ----------------------------
# ⚙️ CẤU HÌNH
# ----------------------------

# Tesseract OCR
pytesseract.pytesseract.tesseract_cmd = r"C:\Program Files\Tesseract-OCR\tesseract.exe"

# SQL Server
conn = pyodbc.connect(
    'DRIVER={ODBC Driver 17 for SQL Server};'
    'SERVER=DESKTOP-PE236IS;'
    'DATABASE=master;'
    'UID=sa;PWD=kien1234'
)
cursor = conn.cursor()

# Serial (ESP32)
ser = serial.Serial('COM4', 115200, timeout=1)  # Cập nhật COM nếu cần

# Load YOLOv8 model
model = YOLO(r"C:\bai_do_xe_thong_minh\runs\detect\train18\weights\best.pt")
model.fuse()

# ----------------------------
# 🧠 AI: Detect & OCR
# ----------------------------
def recognize_plate_yolo8(image):
    results = model.predict(source=image, verbose=False)
    boxes = results[0].boxes.xyxy.cpu().numpy()

    if len(boxes) == 0:
        return None, None

    x1, y1, x2, y2 = map(int, boxes[0])
    plate_crop = image[y1:y2, x1:x2]

    gray = cv2.cvtColor(plate_crop, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    resized = cv2.resize(thresh, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)

    custom_config = r'--oem 3 --psm 8'
    text = pytesseract.image_to_string(resized, config=custom_config)
    text = re.sub(r'\W+', '', text)
    return text.upper(), (x1, y1, x2, y2)

# ----------------------------
# 💾 GHI VÀO DATABASE
# ----------------------------
def insert_plate(plate, direction, temp, humid, flame):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    cursor.execute("""
        INSERT INTO ParkingLogs (plate_number, direction, Timestamp, Temperature, Humidity, Fire)
        VALUES (?, ?, ?, ?, ?, ?)
    """, plate, direction, timestamp, temp, humid, flame)
    conn.commit()

# ----------------------------
# 🔍 KIỂM TRA XE ĐÃ VÀO CHƯA
# ----------------------------
def plate_exists(plate):
    cursor.execute("""
        SELECT
            (SELECT COUNT(*) FROM ParkingLogs WHERE plate_number = ? AND direction = 'in') -
            (SELECT COUNT(*) FROM ParkingLogs WHERE plate_number = ? AND direction = 'out')
    """, plate, plate)
    result = cursor.fetchone()[0]
    return result > 0

# ----------------------------
# 🎬 CHƯƠNG TRÌNH CHÍNH
# ----------------------------
def main():
    cap = cv2.VideoCapture(0)
    print("🔄 Đang chờ ESP32 gửi yêu cầu (REQ_IN / REQ_OUT)...")

    last_temp = None
    last_humid = None
    last_flame = None

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode(errors='ignore').strip()

            # ESP gửi dữ liệu cảm biến
            if line.startswith("DHT:"):
                try:
                    _, values = line.split(":")
                    parts = values.split(",")
                    if len(parts) == 3:
                        last_temp = float(parts[0])
                        last_humid = float(parts[1])
                        last_flame = int(parts[2])  # 0: không cháy, 1: cháy
                        print(f"🌡️ Temp: {last_temp}°C | 💧 Humid: {last_humid}% | 🔥 Flame: {'🔥' if last_flame else '✅'}")
                except Exception as e:
                    print("❌ Lỗi đọc DHT+Flame:", e)
                continue

            # ESP gửi yêu cầu nhận diện xe
            request = line
            print(f"📩 ESP yêu cầu: {request}")

            ret, frame = cap.read()
            if not ret:
                print("❌ Lỗi đọc camera")
                continue

            plate, box = recognize_plate_yolo8(frame)
            if plate:
                print(f"📸 Biển số: {plate}")

                if request == "REQ_IN":
                    insert_plate(plate, "in", last_temp, last_humid, last_flame)
                    ser.write(b'OPEN_IN\n')
                    print("✅ Cho xe vào")

                elif request == "REQ_OUT":
                    if plate_exists(plate):
                        insert_plate(plate, "out", last_temp, last_humid, last_flame)
                        ser.write(b'OPEN_OUT\n')
                        print("✅ Cho xe ra")
                    else:
                        print("❌ Xe chưa vào -> Không mở cổng")
            else:
                print("⚠️ Không nhận diện được biển số")

        # Hiển thị camera nếu muốn
        ret, frame = cap.read()
        if ret:
            cv2.imshow("Smart Parking (Press Q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    conn.close()
    ser.close()

# ----------------------------
# ▶️ CHẠY CHƯƠNG TRÌNH
# ----------------------------
if __name__ == "__main__":
    main()
