package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

import java.util.ArrayList;

public class RobotModules {
    private ArrayList<IRobotModule> modules;

    public Claw claw;

    public RobotModules(HardwareMap hm){
        this.claw = new Claw(hm);
        buildList();
    }

    public void buildList(){
        if(claw.isActive()) modules.add(claw);
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
