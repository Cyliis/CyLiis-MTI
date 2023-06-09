package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.UtaUta;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class TiedBehaviour {
    RobotModules robot;
    DriveTrain driveTrain;
    private final boolean auto;

    NanoClock nanoClock;

    DigitalChannel coneGuideSensor;

    public static boolean coneSensorEnabled = true;

    public TiedBehaviour(RobotModules robot, DriveTrain driveTrain){
        auto = false;
        this.robot = robot;
        this.driveTrain = driveTrain;
        nanoClock = NanoClock.system();
        if(coneSensorEnabled) {
            coneGuideSensor = robot.hm.get(DigitalChannel.class, "coneGuide");
            coneGuideSensor.setMode(DigitalChannel.Mode.INPUT);
        }
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
        if(robot.intake.state == Intake.State.OPENED && !coneGuideSensor.getState() && Virtual.stackIndex == 0){
            robot.intake.setState(Intake.State.CLOSING);
        }
    }

    private void stack(){
        Virtual.State.DOWN.pos_s =  Virtual.stack_s[Virtual.stackIndex];
        Virtual.State.GOING_DOWN.pos_s = Virtual.stack_s[Virtual.stackIndex];
        Virtual.State.DOWN.pos =  Virtual.stack[Virtual.stackIndex];
        Virtual.State.GOING_DOWN.pos = Virtual.stack[Virtual.stackIndex];
        if(Virtual.stackIndex != 0){
            Virtual.State.HOVER.pos = Virtual.hoverPositionStack;
            Virtual.State.GOING_HOVER.pos = Virtual.hoverPositionStack;
            Virtual.State.HOVER.pos_s = Virtual.hoverPositionStack_s;
            Virtual.State.GOING_HOVER.pos_s = Virtual.hoverPositionStack_s;
            UtaUta.State.SMOLANGLEFRONT.pos = UtaUta.levelPosition;
            UtaUta.State.SMOLAGNLINGFRONT.pos = UtaUta.levelPosition;
        }
        else{
            Virtual.State.HOVER.pos = Virtual.hoverPosition;
            Virtual.State.GOING_HOVER.pos = Virtual.hoverPosition;
            Virtual.State.HOVER.pos_s = Virtual.hoverPosition_s;
            Virtual.State.GOING_HOVER.pos_s = Virtual.hoverPosition_s;
            UtaUta.State.SMOLANGLEFRONT.pos = UtaUta.frontBruhPosition;
            UtaUta.State.SMOLAGNLINGFRONT.pos = UtaUta.frontBruhPosition;
        }
    }

    public static double transferTime = 1.4;
    boolean detectedJamTransfer = false;

    private void jamDetectTransfer(){
        if(robot.intake.state == Intake.State.TRANSFERING && nanoClock.seconds() - robot.intake.timeOfLastStateChange >= transferTime && !detectedJamTransfer){
            robot.intake.transferState = Intake.TransferState.ABORT;
            detectedJamTransfer = true;
        }
        if(robot.intake.state != Intake.State.TRANSFERING) detectedJamTransfer = false;
    }

    public static double lowTime = 0.7;
    boolean detectedJamLow = false;

    private void jamDetectLow(){
        if(robot.intake.state == Intake.State.GOING_LOW && nanoClock.seconds() - robot.intake.timeOfLastStateChange >= lowTime && !detectedJamLow){
            robot.intake.setState(Intake.State.LOW);
            detectedJamLow = true;
        }
        if(robot.intake.state != Intake.State.GOING_LOW) detectedJamLow = false;
    }

    private void autoLift(){
        if(robot.intake.state == Intake.State.TRANSFERING
        && (robot.intake.transferState == Intake.TransferState.END || robot.intake.transferState == Intake.TransferState.OPEN_CLAW
                || robot.intake.transferState == Intake.TransferState.VIRTUAL_DOWN_PIVOT_FRONT)
        && robot.outtake.state == Outtake.State.DOWN)  robot.outtake.setState(Outtake.State.GOING_HIGH);

    }

    private void limitSpeedStack(){
        if(Virtual.stackIndex != 0){
            robot.virtual.speedLimit = Virtual.speedLimit2;
        }
        else robot.virtual.speedLimit = Virtual.speedLimit1;
    }

    private void limitRotationStack(){
        if(Virtual.stackIndex != 0){
            robot.virtual.rotatePositionFromFront = Virtual.rotatePositionStack;
        }
        else robot.virtual.rotatePositionFromFront = Virtual.rotatePositionDown;
    }

    public void loop(){
        stack();
//        jamDetectTransfer();
        limitSpeedStack();
        limitRotationStack();
        if(auto) { 
            loopAuto();
            return;
        }
        if(coneSensorEnabled) autoClose();
        hoverCone();
        Lift.State.tele();
    }

    private void loopAuto(){
        Lift.State.auto();
    }
}
