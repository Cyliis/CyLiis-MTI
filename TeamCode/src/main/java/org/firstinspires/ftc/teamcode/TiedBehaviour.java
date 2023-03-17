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
        if(robot.intake.state == Intake.State.OPENED && robot.distanceSensor.value < 37){
            robot.intake.setState(Intake.State.CLOSING);
        }
    }

    public void loop(){
        if(auto) { 
            loopAuto();
            return;
        }
        autoClose();
        hoverCone();
    }

    private void loopAuto(){
        hoverCone();
    }
}
