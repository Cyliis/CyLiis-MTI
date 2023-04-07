package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.UtaUta;
import org.firstinspires.ftc.teamcode.Modules.Virtual;

@Config
public class TiedBehaviour {
    RobotModules robot;
    DriveTrain driveTrain;
    private final boolean auto;

    NanoClock nanoClock;

    public TiedBehaviour(RobotModules robot, DriveTrain driveTrain){
        auto = false;
        this.robot = robot;
        this.driveTrain = driveTrain;
        nanoClock = NanoClock.system();
    }

    public TiedBehaviour(RobotModules robot){
        auto = true;
        this.robot = robot;
        nanoClock = NanoClock.system();
    }

    private void hoverCone(){
        if(robot.intake.state == Intake.State.CLOSED && robot.virtual.state != Virtual.State.HOVER && robot.virtual.state!= Virtual.State.GOING_HOVER) robot.intake.setState(Intake.State.HOVERING);
        if(robot.intake.state == Intake.State.MCLOSED && robot.virtual.state != Virtual.State.HOVER && robot.virtual.state!= Virtual.State.GOING_HOVER) robot.intake.setState(Intake.State.MHOVERING);
    }

    private void autoClose(){
//        if(robot.intake.state == Intake.State.OPENED && robot.distanceSensor.value < 36 && Virtual.stackIndex == 0){
//            robot.intake.setState(Intake.State.CLOSING);
//        }
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

    public static double transferTime = 2;
    boolean detectedJam = false;

    private void jamDetect(){
        if(robot.intake.state == Intake.State.TRANSFERING && nanoClock.seconds() - robot.intake.timeOfLastStateChange >= transferTime && !detectedJam){
            robot.intake.transferState = Intake.TransferState.ABORT;
            detectedJam = true;
        }
        if(robot.intake.state != Intake.State.TRANSFERING) detectedJam = false;
    }

    private void tiltToHover(){
        if(robot.virtual.virtualEncoder.getCurrentPosition() > Virtual.downPositionE + 50 && robot.virtual.virtualEncoder.getCurrentPosition() < Virtual.hoverPositionE - 50)
        {
            UtaUta.State.LEVEL.pos = UtaUta.hoverPosition;
            UtaUta.State.LEVELING.pos = UtaUta.hoverPosition;
        }
        else {
            UtaUta.State.LEVEL.pos = UtaUta.levelPosition;
            UtaUta.State.LEVELING.pos = UtaUta.levelPosition;
        }
    }

    private void autoLift(){
        if(robot.intake.state == Intake.State.TRANSFERING
        && (robot.intake.transferState == Intake.TransferState.END || robot.intake.transferState == Intake.TransferState.OPEN_CLAW
                || robot.intake.transferState == Intake.TransferState.VIRTUAL_DOWN_PIVOT_FRONT)
        && robot.outtake.state == Outtake.State.DOWN)  robot.outtake.setState(Outtake.State.GOING_HIGH);

    }

    public void loop(){
        hoverCone();
        stack();
//        tiltToHover();
        jamDetect();
        if(auto) { 
            loopAuto();
            return;
        }
//        autoClose();
    }

    private void loopAuto(){
        autoLift();
    }
}
