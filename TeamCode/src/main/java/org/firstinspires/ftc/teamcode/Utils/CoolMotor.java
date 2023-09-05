package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class CoolMotor {

    public enum RunMode {
        RUN, PID;
    }

    private final DcMotorEx motor;

    private final PIDController pidController = new PIDController(0,0,0);
    private PIDCoefficients pidCoefficients = new PIDCoefficients(0,0,0);

    private RunMode runMode;

    public CoolMotor(HardwareMap hm, String name, RunMode runMode, boolean reversed){
        this.motor = hm.get(DcMotorEx.class, name);
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(reversed)motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.runMode = runMode;
    }

    public CoolMotor(HardwareMap hm, String name){
        this(hm, name, RunMode.RUN, false);
    }

    public CoolMotor(HardwareMap hm, String name, RunMode runMode){
        this(hm, name, runMode, false);
    }

    public CoolMotor(HardwareMap hm, String name, boolean reversed){
        this(hm, name, RunMode.RUN, false);
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPowerBehaviour){
        motor.setZeroPowerBehavior(zeroPowerBehaviour);
    }

    public void setMode(RunMode runMode){
        this.runMode = runMode;
    }

    private double power;

    public void setPower(double power){
        if(runMode == RunMode.PID) return;
        this.power = power;
    }

    public void setPID(PIDCoefficients pidCoefficients){
        this.pidCoefficients = pidCoefficients;
    }

    private double feedforward;

    public void setPIDF(PIDCoefficients pidCoefficients, double feedforward){
        this.feedforward = feedforward;
        this.pidCoefficients.p = pidCoefficients.p;
        this.pidCoefficients.i = pidCoefficients.i;
        this.pidCoefficients.d = pidCoefficients.d;
    }

    public void setPIDF(PIDFCoefficients pidfCoefficients, double feedforward){
        this.feedforward = feedforward;
        pidCoefficients.p = pidfCoefficients.p;
        pidCoefficients.i = pidfCoefficients.i;
        pidCoefficients.d = pidfCoefficients.d;
    }

    public void calculatePower(double current, double target){
        if(runMode == RunMode.RUN) return;
        pidController.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);
        power = feedforward + pidController.calculate(current,target);
    }

    public void update(){
        motor.setPower(power);
    }

}
