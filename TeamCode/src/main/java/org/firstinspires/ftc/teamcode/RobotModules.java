package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.DistanceSensor;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.Modules.VirtualPivot;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

import java.util.ArrayList;

@Config
public class RobotModules {
    private ArrayList<IRobotModule> modules = new ArrayList<>();

    public Latch latch;
    public Claw claw;
    public Virtual virtual;
    public Intake intake;
    public Outtake outtake;

    public RobotModules(HardwareMap hm, boolean resetEncoders){
        if(Latch.ENABLE_MODULE)this.latch = new Latch(hm);
        if(Claw.ENABLE_MODULE)this.claw = new Claw(hm);
        if(Virtual.ENABLE_MODULE)this.virtual = new Virtual(hm, resetEncoders);
        if(Intake.ENABLE_MODULE)this.intake = new Intake(hm, latch, virtual, claw);
        if(Outtake.ENABLE_MODULE)this.outtake = new Outtake(hm, latch, virtual, claw, resetEncoders);
        buildList();
    }

    public void telemetry(Telemetry tele){
        tele.addData("Latch state",latch.state);
        tele.addData("Intake state", intake.state);
        tele.addData("Outtake state", outtake.state);
        tele.addData("Transfer state", intake.transferState);
        tele.addData("Claw state", intake.claw.state);
        tele.addData("Lift state", outtake.lift.state);
        tele.addData("Uta state", intake.uta.state);
        tele.addData("Virtual state", virtual.state);
        tele.addData("Pivot state", intake.pivot.state);
//        tele.addData("Stack index", Virtual.stackIndex);
//        tele.addData("Lift1 current position", outtake.lift.lift1.getCurrentPosition());
//        tele.addData("Lift2 current position", outtake.lift.lift2.getCurrentPosition());
//        tele.addData("Lift target position", outtake.lift.state.pos + outtake.lift.ground);
//        tele.addData("Lift ground position", outtake.lift.ground);
        tele.addData("Lift1 power draw", outtake.lift.lift1.getCurrent(CurrentUnit.AMPS));
        tele.addData("Lift2 power draw", outtake.lift.lift2.getCurrent(CurrentUnit.AMPS));
    }

    public void buildList(){
        if(Latch.ENABLE_MODULE) modules.add(latch);
        if(Claw.ENABLE_MODULE) modules.add(claw);
        if(Virtual.ENABLE_MODULE) modules.add(virtual);
        if(Intake.ENABLE_MODULE) modules.add(intake);
        if(Outtake.ENABLE_MODULE) modules.add(outtake);
    }

    public void atStart(){
        for(IRobotModule module : modules){
            module.atStart();
        }
    }

    public void loop(){
        for(IRobotModule module : modules){
            module.loop();
        }
    }

    public void emergencyStop(){
        for(IRobotModule module : modules){
            module.emergencyStop();
        }
    }
}
