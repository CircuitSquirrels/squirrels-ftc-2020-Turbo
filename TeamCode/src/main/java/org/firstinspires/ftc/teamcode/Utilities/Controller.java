package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
Thanks to cporter!
https://github.com/cporter/
 */

public class Controller {
    private Gamepad gamepad;

    private int dpad_up, dpad_down, dpad_left, dpad_right;
    private int x, y, a, b, start;
    private int left_bumper, right_bumper;
    private int right_stick_button, left_stick_button;

    public double left_stick_x, right_stick_x, left_stick_y, right_stick_y;
    public double left_trigger, right_trigger;

    public Controller(Gamepad g) {
        gamepad = g;
    }

    public void update() {
        if (gamepad.right_stick_button) {
            ++right_stick_button;
        } else {
            right_stick_button = 0;
        }
        if (gamepad.left_stick_button) {
            ++left_stick_button;
        } else {
            left_stick_button = 0;
        }
        if (gamepad.x) {
            ++x;
        } else {
            x = 0;
        }
        if (gamepad.y) {
            ++y;
        } else {
            y = 0;
        }
        if (gamepad.a) {
            ++a;
        } else {
            a = 0;
        }
        if (gamepad.b) {
            ++b;
        } else {
            b = 0;
        }
        if (gamepad.start) {
            ++start;
        } else {
            start = 0;
        }
        if (gamepad.dpad_up) {
            ++dpad_up;
        } else {
            dpad_up = 0;
        }
        if (gamepad.dpad_down) {
            ++dpad_down;
        } else {
            dpad_down = 0;
        }
        if (gamepad.dpad_left) {
            ++dpad_left;
        } else {
            dpad_left = 0;
        }
        if (gamepad.dpad_right) {
            ++dpad_right;
        } else {
            dpad_right = 0;
        }
        if (gamepad.left_bumper) {
            ++left_bumper;
        } else {
            left_bumper = 0;
        }
        if (gamepad.right_bumper) {
            ++right_bumper;
        } else {
            right_bumper = 0;
        }

        left_stick_x = gamepad.left_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_x = gamepad.right_stick_x;
        right_stick_y = gamepad.right_stick_y;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }

    public boolean rightStickButton() {
        return  0 < right_stick_button;
    }

    public boolean leftStickButton() {
        return  0 < left_stick_button;
    }

    public boolean dpadUp() {
        return 0 < dpad_up;
    }

    public boolean dpadDown() {
        return 0 < dpad_down;
    }

    public boolean dpadLeft() {
        return 0 < dpad_left;
    }

    public boolean dpadRight() {
        return 0 < dpad_right;
    }

    public boolean X() {
        return 0 < x;
    }

    public boolean Y() {
        return 0 < y;
    }

    public boolean A() {
        return 0 < a;
    }

    public boolean B() {
        return 0 < b;
    }

    public boolean start() {
        return 0 < start;
    }

    public boolean leftBumper() {
        return 0 < left_bumper;
    }

    public boolean rightBumper() {
        return 0 < right_bumper;
    }

    public boolean dpadUpOnce() {
        return 1 == dpad_up;
    }

    public boolean dpadDownOnce() {
        return 1 == dpad_down;
    }

    public boolean dpadLeftOnce() {
        return 1 == dpad_left;
    }

    public boolean dpadRightOnce() {
        return 1 == dpad_right;
    }

    public boolean rightStickButtonOnce() {
        return 1 == right_stick_button;
    }

    public boolean leftStickButtonOnce() {
        return 1 == left_stick_button;
    }

    public boolean XOnce() {
        return 1 == x;
    }

    public boolean YOnce() {
        return 1 == y;
    }

    public boolean AOnce() {
        return 1 == a;
    }

    public boolean BOnce() {
        return 1 == b;
    }

    public boolean startOnce() {
        return 1 == start;
    }

    public boolean leftBumperOnce() {
        return 1 == left_bumper;
    }

    public boolean rightBumperOnce() {
        return 1 == right_bumper;
    }

    static public double deadZone(double input,double deadzone) {
        if (Math.abs(input) <= deadzone) {
            return 0.0;
        } else {
            double sign = input/Math.abs(input);
            double output = 1/(1-deadzone)*(Math.abs(input)-deadzone);
            return sign * output;
        }
    }


}