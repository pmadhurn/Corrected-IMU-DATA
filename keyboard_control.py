from pynput import keyboard
from sp2520_controller import SP2520Controller
import serial

def keyboard_control(controller):
    pressed_keys = set()
    
    def update_movement():
        if not pressed_keys:
            controller.stop()
        elif 'w' in pressed_keys and 'a' in pressed_keys:
            controller.move_up_left()
        elif 'w' in pressed_keys and 'd' in pressed_keys:
            controller.move_up_right()
        elif 's' in pressed_keys and 'a' in pressed_keys:
            controller.move_down_left()
        elif 's' in pressed_keys and 'd' in pressed_keys:
            controller.move_down_right()
        elif 'w' in pressed_keys:
            controller.move_up()
        elif 's' in pressed_keys:
            controller.move_down()
        elif 'a' in pressed_keys:
            controller.move_left()
        elif 'd' in pressed_keys:
            controller.move_right()
    
    def on_press(key):
        if hasattr(key, 'char'):
            if key.char in ['w', 'a', 's', 'd']:
                pressed_keys.add(key.char)
                update_movement()
            elif key.char == 'q':
                new_speed = max(5, controller.pan_speed - 5)
                controller.set_speed(new_speed, new_speed)
            elif key.char == 'e':
                new_speed = min(0x40, controller.pan_speed + 5)
                controller.set_speed(new_speed, new_speed)
            elif key.char in '123456789':
                controller.call_preset(int(key.char))
    
    def on_release(key):
        if key == keyboard.Key.esc:
            controller.close()
            return False
        if hasattr(key, 'char') and key.char in pressed_keys:
            pressed_keys.remove(key.char)
            update_movement()
    
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

if __name__ == "__main__":
    controller = SP2520Controller(port='/dev/ttyUSB0', baudrate=2400, address=1)
    print("Controls: WASD=Move, Q/E=Speed, 1-9=Presets, ESC=Exit")
    keyboard_control(controller)