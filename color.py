
import cv2
import numpy as np
import serial
import time

# --- CONFIGURATION ---
CAMERA_URL = "http://10.223.225.79:81/stream"
SERIAL_PORT = "COM3"      # UPDATE WITH YOUR ARDUINO COM PORT
BAUD_RATE = 115200

# HSV Ranges and Visualization Colors
COLORS = {
    "Red": {
        "ranges": [(np.array([0, 100, 60]), np.array([10, 255, 255])), 
                   (np.array([160, 100, 60]), np.array([179, 255, 255]))],
        "draw_color": (0, 0, 255),
        "char": 'r'
    },
    "Green": {
        "ranges": [(np.array([45, 60, 50]), np.array([85, 255, 255]))],
        "draw_color": (0, 255, 0),
        "char": 'g'
    },
    "Blue": {
        "ranges": [(np.array([100, 80, 50]), np.array([130, 255, 255]))],
        "draw_color": (255, 0, 0),
        "char": 'b'
    }
}

def process_frame(frame):
    """Analyzes the frame, draws bounding boxes, and returns detected color name and char."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detected_char = None
    detected_name = "None"
    
    for name, info in COLORS.items():
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for r in info["ranges"]:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, r[0], r[1]))
        
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_cnt = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_cnt) > 3000:
                x, y, w, h = cv2.boundingRect(largest_cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), info["draw_color"], 2)
                cv2.putText(frame, name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, info["draw_color"], 2)
                
                detected_char = info["char"]
                detected_name = name
                
    return frame, detected_char, detected_name

# --- MAIN LOOP ---
try:
    print(f"Connecting to Arduino on {SERIAL_PORT}...")
    ard = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print("Opening Camera Stream...")
    cap = cv2.VideoCapture(CAMERA_URL)
    
    time.sleep(2)
    print("--- SYSTEM READY ---")

    while True:
        ret, frame = cap.read()
        if not ret:
            cap = cv2.VideoCapture(CAMERA_URL)
            continue

        # Live detection for the display window
        processed_frame, live_char, live_name = process_frame(frame)

        # Check if Arduino is requesting a color detection
        if ard.in_waiting:
            line = ard.readline().decode('utf-8', errors='ignore').strip()
            
            if "START_VISION" in line:
                print("\n[SIGNAL] Conveyor Stopped. Capturing frame...")
                
                # Clear buffer for the most accurate reading
                for _ in range(5): cap.grab()
                ret, frame = cap.read()
                
                _, final_char, final_name = process_frame(frame)
                
                # --- PRINTING THE IDENTIFIED COLOR ---
                if final_char:
                    print(f"[*] IDENTIFIED COLOR: {final_name.upper()}")
                    print(f"[*] SENDING '{final_char}' TO ARDUINO\n")
                    ard.write(final_char.encode())
                else:
                    print("[!] WARNING: No color detected. Defaulting to Red.")
                    ard.write(b'r')

        cv2.imshow("Robot Vision System", processed_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except Exception as e:
    print(f"Error: {e}")

finally:
    if 'ard' in locals(): ard.close()
    if 'cap' in locals(): cap.release()
    cv2.destroyAllWindows()
