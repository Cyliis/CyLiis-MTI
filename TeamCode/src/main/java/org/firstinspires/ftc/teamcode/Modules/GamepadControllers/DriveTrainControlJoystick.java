package org.firstinspires.ftc.teamcode.Modules.GamepadControllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@Config
public class DriveTrainControlJoystick {
    Gamepad gamepad1, gamepad2;
    StickyGamepad stickyGamepad1,stickyGamepad2;
    DriveTrain dt;

    public DriveTrainControlJoystick(Gamepad gamepad1, Gamepad gamepad2, DriveTrain dt, RobotModules robot){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.stickyGamepad1 = new StickyGamepad(gamepad1);
        this.stickyGamepad2 = new StickyGamepad(gamepad2);
        this.dt = dt;
    }

    private void drive(){
        dt.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    private void switchMode(){
        if(stickyGamepad1.y) {
            switch (dt.mode){
                case FIELD_CENTRIC:
                    dt.mode = DriveTrain.DRIVE_MODE.ROBOT_CENTRIC;
                    break;
                case ROBOT_CENTRIC:
                    dt.mode = DriveTrain.DRIVE_MODE.FIELD_CENTRIC;
                    break;
            }
        }
    }

    private void speedControl(){
        if(gamepad1.b) dt.speed = DriveTrain.SPEED.SLOW;
        else dt.speed = DriveTrain.SPEED.FAST;
    }

    private void resetHeding(){
        if(gamepad1.left_bumper && gamepad1.right_bumper) DriveTrain.imuOffset = dt.imuValue;
    }

    public void loop(){
        stickyGamepad1.update();
        stickyGamepad2.update();
        drive();
        switchMode();
        speedControl();
        resetHeding();
    }
}
