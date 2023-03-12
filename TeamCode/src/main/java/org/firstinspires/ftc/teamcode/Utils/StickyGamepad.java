package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class StickyGamepad extends Gamepad{
    Gamepad lastState = new Gamepad();

    public void update(Gamepad uGamepad){
        this.touchpad_finger_1 = !lastState.touchpad_finger_1 && uGamepad.touchpad_finger_1;
        this.touchpad_finger_2 = !lastState.touchpad_finger_2 && uGamepad.touchpad_finger_2;
        this.touchpad = !lastState.touchpad && uGamepad.touchpad;
        this.left_stick_button = !lastState.left_stick_button && uGamepad.left_stick_button;
        this.right_stick_button = !lastState.right_stick_button && uGamepad.right_stick_button;
        this.dpad_up = !lastState.dpad_up && uGamepad.dpad_up;
        this.dpad_down = !lastState.dpad_down && uGamepad.dpad_down;
        this.dpad_left = !lastState.dpad_left && uGamepad.dpad_left;
        this.dpad_right = !lastState.dpad_right && uGamepad.dpad_right;
        this.a = !lastState.a && uGamepad.a;
        this.b = !lastState.b && uGamepad.b;
        this.x = !lastState.x && uGamepad.x;
        this.y = !lastState.y && uGamepad.y;
        this.guide = !lastState.guide && uGamepad.guide;
        this.start = !lastState.start && uGamepad.start;
        this.back = !lastState.back && uGamepad.back;
        this.left_bumper = !lastState.left_bumper && uGamepad.left_bumper;
        this.right_bumper = !lastState.right_bumper && uGamepad.right_bumper;

        this.lastState = uGamepad;
    }
}
