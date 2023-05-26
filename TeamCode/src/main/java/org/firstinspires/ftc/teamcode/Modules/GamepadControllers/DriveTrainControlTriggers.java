package org.firstinspires.ftc.teamcode.Modules.GamepadControllers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@Config
public class DriveTrainControlTriggers {
    Gamepad gamepad1, gamepad2;
    RobotModules robot;
    StickyGamepad stickyGamepad1,stickyGamepad2;
    DriveTrain dt;
    NanoClock nanoClock;

    public DriveTrainControlTriggers(Gamepad gamepad1, Gamepad gamepad2, DriveTrain dt, RobotModules robot){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.stickyGamepad1 = new StickyGamepad(gamepad1);
        this.stickyGamepad2 = new StickyGamepad(gamepad2);
        this.dt = dt;
        this.robot = robot;
        nanoClock = NanoClock.system();
    }

    private void drive(){
        dt.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.left_trigger + gamepad1.right_trigger);
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

    public static double movement = 1, timeout = 0.2;

    private void moveWhenLiftComingDown(){
        if(robot.outtake.state == Outtake.State.GOING_DOWN){
            if(nanoClock.seconds() - robot.outtake.timeOfLastStateChange < timeout){
                dt.drive(movement * (1/dt.speed.multiplier), 0,0);
            }
        }
    }

    public void loop(){
        stickyGamepad1.update();
        stickyGamepad2.update();
        drive();
        switchMode();
        speedControl();
        resetHeding();
//        moveWhenLiftComingDown();
    }
}
