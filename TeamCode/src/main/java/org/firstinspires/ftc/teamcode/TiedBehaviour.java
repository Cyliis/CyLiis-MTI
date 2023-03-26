package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;

@Config
public class TiedBehaviour {
    RobotModules robot;
    DriveTrain driveTrain;
    private final boolean auto;

    public TiedBehaviour(RobotModules robot, DriveTrain driveTrain){
        auto = false;
        this.robot = robot;
        this.driveTrain = driveTrain;
    }

    public TiedBehaviour(RobotModules robot){
        auto = true;
        this.robot = robot;
    }

    private void hoverCone(){
        if(robot.intake.state == Intake.State.CLOSED && robot.virtual.state != Virtual.State.HOVER && robot.virtual.state!= Virtual.State.GOING_HOVER) robot.intake.setState(Intake.State.HOVERING);
        if(robot.intake.state == Intake.State.MCLOSED && robot.virtual.state != Virtual.State.HOVER && robot.virtual.state!= Virtual.State.GOING_HOVER) robot.intake.setState(Intake.State.MHOVERING);
    }

    private void autoClose(){
        if(robot.intake.state == Intake.State.OPENED && robot.distanceSensor.value < 36 && Virtual.stackIndex == 0){
            robot.intake.setState(Intake.State.CLOSING);
        }
    }

    private void stack(){
        Virtual.State.DOWN.pos =  Virtual.stack[Virtual.stackIndex];
        Virtual.State.GOING_DOWN.pos = Virtual.stack[Virtual.stackIndex];
        Virtual.State.DOWN.encPos =  Virtual.stackE[Virtual.stackIndex];
        Virtual.State.GOING_DOWN.encPos = Virtual.stackE[Virtual.stackIndex];
        if(Virtual.stackIndex != 0){
            Virtual.State.HOVER.pos = Virtual.lowPosition;
            Virtual.State.GOING_HOVER.pos = Virtual.lowPosition;
        }
        else{
            Virtual.State.HOVER.pos = Virtual.hoverPosition;
            Virtual.State.GOING_HOVER.pos = Virtual.hoverPosition;
        }
    }

    public void loop(){
        hoverCone();
        stack();
        if(auto) { 
            loopAuto();
            return;
        }
        autoClose();
    }

    private void loopAuto(){
    }
}
