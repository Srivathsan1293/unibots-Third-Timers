from controller import Keyboard

def move_from_key_input(keyboard, speed):
    key = keyboard.getKey()

    left = 0
    right = 0

    if key == Keyboard.UP:
        left = speed
        right = speed

    elif key == Keyboard.DOWN:
        left = -speed
        right = -speed

    elif key == Keyboard.LEFT:
        left = -speed
        right = speed

    elif key == Keyboard.RIGHT:
        left = speed
        right = -speed

    return left, right
