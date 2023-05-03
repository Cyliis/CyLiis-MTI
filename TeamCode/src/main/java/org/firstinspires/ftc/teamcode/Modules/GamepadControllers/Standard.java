package org.firstinspires.ftc.teamcode.Modules.GamepadControllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@Config
public class Standard {
    Gamepad gamepad1, gamepad2;
    StickyGamepad stickyGamepad1,stickyGamepad2;
    RobotModules robot;

    public Standard(Gamepad gamepad1, Gamepad gamepad2, RobotModules robot){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.stickyGamepad1 = new StickyGamepad(gamepad1);
        this.stickyGamepad2 = new StickyGamepad(gamepad2);
        this.robot = robot;
    }

    private void updateStickyGamepads(){
        stickyGamepad1.update();
        stickyGamepad2.update();
    }

    private void intakeGamepadControl(){
        if(!Intake.ENABLE_MODULE) return;
        if(stickyGamepad1.x &&
                (robot.intake.virtual.state == Virtual.State.DOWN || robot.virtual.state == Virtual.State.HOVER || robot.virtual.state == Virtual.State.GOING_HOVER)
        && robot.outtake.state == Outtake.State.DOWN) robot.intake.setState(Intake.State.TRANSFERING);
        if(stickyGamepad1.a){
            switch (robot.intake.state){
                case CLOSED:
                case CLOSING:
                case MCLOSED:
                case MCLOSING:
                case MHOVERING:
                case HOVERING:
                    robot.intake.setState(Intake.State.MOPENING);
                    break;
                case OPENED:
                case OPENING:
                case MOPENED:
                case MOPENING:
                    robot.intake.setState(Intake.State.MCLOSING);
                    break;
            }
        }
        if(stickyGamepad1.dpad_down){
            robot.intake.transferState = Intake.TransferState.ABORT;
        }
        if(stickyGamepad2.a){
            if(robot.intake.state == Intake.State.LOW) robot.intake.setState(Intake.State.REALEASING_LOW);
        }
        if(stickyGamepad2.dpad_down){
            if(robot.intake.state == Intake.State.LOW || robot.intake.state == Intake.State.RELEASED_LOW){
                robot.intake.setState(Intake.State.GOING_DOWN_FROM_LOW);
            }
            else if(robot.intake.state!= Intake.State.TRANSFERING) robot.intake.setState(Intake.State.GOING_LOW);
        }
    }

    private void outtakeGamepadControl(){
        if(!Outtake.ENABLE_MODULE)return;
        if(robot.intake.state== Intake.State.GOING_LOW || robot.intake.state == Intake.State.LOW
                || robot.intake.state== Intake.State.RELEASED_LOW || robot.intake.state== Intake.State.GOING_DOWN_FROM_LOW)return;
        if(robot.intake.transferState != Intake.TransferState.END && robot.intake.transferState != Intake.TransferState.OPEN_CLAW && robot.intake.transferState != Intake.TransferState.VIRTUAL_DOWN_PIVOT_FRONT) return;
        if(stickyGamepad2.x) robot.outtake.setState(Outtake.State.GOING_DOWN);
        if(stickyGamepad2.dpad_left) robot.outtake.setState(Outtake.State.GOING_DOWN_NO_LATCH);
        if(stickyGamepad2.dpad_right) robot.outtake.setState(Outtake.State.GOING_MID);
        if(stickyGamepad2.dpad_up) robot.outtake.setState(Outtake.State.GOING_HIGH);
    }

    private void sensor(){
        if(stickyGamepad1.dpad_left){
            if(robot.intake.state == Intake.State.MOPENED) robot.intake.state = Intake.State.OPENED;
            if(robot.intake.state == Intake.State.MOPENING) robot.intake.state = Intake.State.OPENING;
        }

    }

    private void stackControl(){
        if(stickyGamepad2.left_bumper){
            if(Virtual.stackIndex > 0) Virtual.stackIndex--;
        }
        if(stickyGamepad2.right_bumper){
            if(Virtual.stackIndex < 4) Virtual.stackIndex++;
        }
    }

    private void manualCalibration(){
        if(gamepad2.y) robot.outtake.lift.ground++;
        if(gamepad2.b) robot.outtake.lift.ground--;
    }

    public void loop(){
        intakeGamepadControl();
        outtakeGamepadControl();
        sensor();
        stackControl();
        manualCalibration();


        updateStickyGamepads();
    }
    
}
