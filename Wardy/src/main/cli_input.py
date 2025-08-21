import serial
import keyboard
import time
import os
import sys

SERIAL_PORT = 'COM7'  
BAUD_RATE = 115200

# Servo and gripper state tracking
base_angle = 90  
link_angle = 90
gripper_state = 'Closed'  
DUTY_STEP = 5  
DUTY_RANGE = 48  
ANGLE_PER_DUTY = 180 / DUTY_RANGE  

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def print_interface(pressed_key=None):
    clear_screen()
    print("=== 2 DOF Robot CLI Control ===")
    print(f"Base Angle: {base_angle:.1f}° | Link Angle: {link_angle:.1f}° | Gripper: {gripper_state}")
    print("\nControls:")
    print("  W: Base clockwise | S: Base counterclockwise")
    print("  A: Link up        | D: Link down")
    print("  O: Open gripper   | C: Close gripper")
    print("  Q: Blink LED (and exit)")
    print("\nLayout:")
    
    # Highlight pressed key in green, others in default
    w_color = '\033[92m' if pressed_key == 'w' else '\033[0m'
    s_color = '\033[92m' if pressed_key == 's' else '\033[0m'
    a_color = '\033[92m' if pressed_key == 'a' else '\033[0m'
    d_color = '\033[92m' if pressed_key == 'd' else '\033[0m'
    o_color = '\033[92m' if pressed_key == 'o' else '\033[0m'
    c_color = '\033[92m' if pressed_key == 'c' else '\033[0m'
    q_color = '\033[92m' if pressed_key == 'q' else '\033[0m'
    
    print(f"     {w_color}[W]\033[0m")
    print(f"{a_color}[A]\033[0m   {s_color}[S]\033[0m   {d_color}[D]\033[0m")
    print(f"{o_color}[O]\033[0m   {c_color}[C]\033[0m   {q_color}[Q]\033[0m")
    print("\nPress keys to control. Hold for continuous movement. Ctrl+C or Q to exit.")

def main():
    global base_angle, link_angle, gripper_state
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)
    
    print_interface()
    
    try:
        while True:

            for key in ['w', 's', 'a', 'd', 'o', 'c', 'q']:
                if keyboard.is_pressed(key):
                    # Send command to ESP32
                    ser.write(key.encode())
                    
                    # Update local state for display
                    if key == 'w' and base_angle < 180:
                        base_angle += DUTY_STEP * ANGLE_PER_DUTY
                    elif key == 's' and base_angle > 0:
                        base_angle -= DUTY_STEP * ANGLE_PER_DUTY
                    elif key == 'a' and link_angle < 180:
                        link_angle += DUTY_STEP * ANGLE_PER_DUTY
                    elif key == 'd' and link_angle > 0:
                        link_angle -= DUTY_STEP * ANGLE_PER_DUTY
                    elif key == 'o':
                        gripper_state = 'Open'
                    elif key == 'c':
                        gripper_state = 'Closed'
                    elif key == 'q':
                        print_interface(key)
                        time.sleep(0.1)  
                        ser.close()
                        sys.exit(0)
                    
                    print_interface(key)
                    time.sleep(0.1)  # Debouncing
                else:
                    print_interface()
            
            time.sleep(0.01)  # Prevent CPU overload
            
    except KeyboardInterrupt:
        print("\nExiting...")
        ser.close()
        sys.exit(0)

if __name__ == '__main__':
    main()