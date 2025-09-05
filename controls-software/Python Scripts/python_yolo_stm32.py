#!/usr/bin/env python3
"""
YOLO Object Detection with STM32 Communication
Sends center coordinates and bounding box dimensions to STM32
"""

import cv2
from ultralytics import YOLO
import numpy as np
import serial
import time
import sys

def list_available_ports():
    """List all available serial ports"""
    import serial.tools.list_ports
    
    ports = serial.tools.list_ports.comports()
    if ports:
        print("Available serial ports:")
        for port in ports:
            print(f"  - {port.device}")
    else:
        print("No serial ports found")
    
    return [port.device for port in ports]

def main():
    print("YOLO Object Detection with STM32 Communication")
    print("=" * 50)
    
    # List available ports
    available_ports = list_available_ports()
    
    if not available_ports:
        print("No serial ports available. Check your USB-to-Serial converter.")
        return
    
    # Get port from user or command line
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = input(f"\nEnter serial port (e.g., {available_ports[0]}): ").strip()
        if not port:
            port = available_ports[0]
    
    # Connect to STM32
    try:
        ser = serial.Serial(
            port=port,
            baudrate=9600,
            timeout=1.0,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        
        # Wait for connection to stabilize
        time.sleep(2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print(f"Connected to STM32 on {port}")
        
        # Listen for startup message
        time.sleep(1)
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            if response:
                print(f"STM32: {response}")
        
        # Load YOLO model
        print("Loading YOLO model...")
        model = YOLO('yolov8n.pt')
        
        # Initialize camera
        cap = cv2.VideoCapture(1)
        cap.set(3, 1280)
        cap.set(4, 720)
        
        cv2.namedWindow("Object Detection")
        
        print("Starting object detection... Press 'q' to quit")
        
        while True:
            success, img = cap.read()
            if not success:
                continue
            
            # Run object detection
            results = model(img, verbose=False)
            
            # Extract detections
            if results[0].boxes is not None:
                boxes = results[0].boxes.xyxy.cpu().numpy()
                classes = results[0].boxes.cls.cpu().numpy()
                
                for box, cls in zip(boxes, classes):
                    x1, y1, x2, y2 = box.astype(int)
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # Calculate bounding box dimensions
                    width = x2 - x1
                    height = y2 - y1
                    
                    # Draw rectangle and center point
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)
                    
                    # Get class name
                    class_name = model.names[int(cls)]
                    cv2.putText(img, f'{class_name}', (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    
                    # Display info on image
                    cv2.putText(img, f'C({center_x},{center_y})', (x1, y1-30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
                    cv2.putText(img, f'({width},{height})', (x1, y1-50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
                    
                    # Send data to STM32
                    try:
                        message = f"{center_x},{center_y},{width},{height}\r"
                        ser.write(message.encode('utf-8'))
                        ser.flush()
                        
                        # Check for STM32 response
                        if ser.in_waiting > 0:
                            response = ser.readline().decode('utf-8').strip()
                            if response:
                                print(f"STM32: {response}")
                                
                    except Exception as e:
                        print(f"Serial communication error: {e}")
                        break
            
            cv2.imshow("Object Detection", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
    except serial.SerialException as e:
        print(f"Failed to connect to {port}: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        try:
            ser.close()
            print("Disconnected from STM32")
        except:
            pass

if __name__ == "__main__":
    main()