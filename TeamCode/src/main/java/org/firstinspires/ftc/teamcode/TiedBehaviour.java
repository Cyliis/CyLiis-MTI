package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepadGenerator;

public class TiedBehaviour {
    RobotModules robot;
    DriveTrain driveTrain;
    StickyGamepadGenerator sGGamepad1, sGGamepad2;
    Gamepad gamepad1, gamepad2;
    Gamepad sGamepad1, sGamepad2;
    private final boolean auto;

    public TiedBehaviour(Gamepad gamepad1, Gamepad gamepad2, RobotModules robot, DriveTrain driveTrain){
        auto = false;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.robot = robot;
        this.driveTrain = driveTrain;
    }

    public TiedBehaviour(RobotModules robot){
        auto = true;
        this.robot = robot;
    }

    private void updateStickyGamepads(){
        sGamepad1 = sGGamepad1.update(gamepad1);
        sGamepad2 = sGGamepad2.update(gamepad2);
    }

    private void clawGamepadControl(){
        if(!robot.claw.isActive()) return;
        if(sGamepad1.x){
            if(robot.claw.state == Claw.State.CLOSED || robot.claw.state == Claw.State.CLOSING || robot.claw.state == Claw.State.MCLOSED) robot.claw.setState(Claw.State.OPENING);
            else if(robot.claw.state == Claw.State.OPENED || robot.claw.state == Claw.State.OPENING || robot.claw.state == Claw.State.MOPENED) robot.claw.setState(Claw.State.CLOSING);
        }
    }

    public void loop(){
        if(auto) {
            loopAuto();
            return;
        }
        updateStickyGamepads();
        clawGamepadControl();
    }

    private void loopAuto(){

    }
}
