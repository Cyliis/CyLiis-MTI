package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DumbEncoder {
    private DcMotorEx motor;
    private boolean reversed = false;

    public DumbEncoder(HardwareMap hm, String hmName, boolean reversed){
        this.motor = hm.get(DcMotorEx.class, hmName);
        this.reversed = reversed;
    }

    public DumbEncoder(HardwareMap hm, String hmName){
        this.motor = hm.get(DcMotorEx.class, hmName);
        this.reversed = false;
    }

    public int getCurrentPosition(){
        if(reversed) return -motor.getCurrentPosition();
        return motor.getCurrentPosition();
    }

    public double getVelocity(){
        if(reversed) return -motor.getVelocity();
        else return motor.getVelocity();
    }

    public void reset(){
        DcMotorEx.RunMode prev = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(prev);
    }

    public void setReversed(boolean rev){
        this.reversed = rev;
    }
}
