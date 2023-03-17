package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@Config
public class GamepadControl {
    Gamepad gamepad1, gamepad2;
    StickyGamepad stickyGamepad1,stickyGamepad2;
    RobotModules robot;

    public GamepadControl(Gamepad gamepad1, Gamepad gamepad2, RobotModules robot){
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
        if(stickyGamepad2.a && robot.intake.transferState == Intake.TransferState.END && robot.outtake.state == Outtake.State.DOWN
                && robot.intake.state!= Intake.State.GOING_LOW && robot.intake.state != Intake.State.LOW
        && robot.intake.state!= Intake.State.RELEASED_LOW && robot.intake.state!= Intake.State.GOING_DOWN_FROM_LOW) robot.intake.setState(Intake.State.TRANSFERING);
        if(stickyGamepad1.a){
            switch (robot.intake.state){
                case CLOSED:
                case CLOSING:
                case MCLOSED:
                case MCLOSING:
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
        if(robot.intake.transferState != Intake.TransferState.END && robot.intake.transferState != Intake.TransferState.OPEN_CLAW && robot.intake.transferState != Intake.TransferState.VIRTUAL_DOWN) return;
        if(stickyGamepad2.dpad_right || stickyGamepad2.dpad_left) robot.outtake.setState(Outtake.State.GOING_MID);
        if(stickyGamepad2.dpad_up) robot.outtake.setState(Outtake.State.GOING_HIGH);
        if(stickyGamepad2.x) robot.outtake.setState(Outtake.State.GOING_DOWN);
    }

    private void sensor(){
        if(stickyGamepad1.dpad_left){
            if(robot.intake.state == Intake.State.MOPENED) robot.intake.state = Intake.State.OPENED;
            if(robot.intake.state == Intake.State.MOPENING) robot.intake.state = Intake.State.OPENING;
        }

    }

    private void stackControl(){
        if(stickyGamepad1.left_bumper){
            if(Virtual.stackIndex > 0) Virtual.stackIndex--;
        }
        if(stickyGamepad1.right_bumper){
            if(Virtual.stackIndex < 4) Virtual.stackIndex++;
        }
        Virtual.State.DOWN.pos =  Virtual.stack[Virtual.stackIndex];
        Virtual.State.GOING_DOWN.pos = Virtual.stack[Virtual.stackIndex];
        Virtual.State.HOVER.pos = Virtual.lowPosition;
        Virtual.State.GOING_HOVER.pos = Virtual.lowPosition;
    }

    public void loop(){
        intakeGamepadControl();
        outtakeGamepadControl();
        sensor();
        stackControl();
        updateStickyGamepads();
    }
    
}
