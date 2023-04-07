package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Lift implements IRobotModule {

    public static boolean ENABLE_MODULE = true;


    public static String LIFT1_NAME = "lift1";
    public static String LIFT2_NAME = "lift2";
    public static boolean reversed1 = true, reversed2 = false;
    public int ground = 0;

    HardwareMap hm;
    NanoClock nanoClock;

    public DcMotorEx lift1, lift2;

    public static int downPosition = 0, lowPosition = downPosition, midPosition = 540, highPosition=925;
    public int target = downPosition;
    public static double liftPower = 1;

    public static double p = 0.000028,i = 0.001,d = 0.000028;
    public static double f = 0;


    PIDController pid = new PIDController(p,i,d);

    public enum State{
        DOWN(downPosition),
        LOW(lowPosition),
        MID(midPosition),
        HIGH(highPosition),
        GOING_DOWN(downPosition),
        GOING_LOW(lowPosition),
        GOING_MID(midPosition),
        GOING_HIGH(highPosition);

        public int pos;
        State(int pos){this.pos = pos;}
    }

    public State state;

    public Lift(HardwareMap hm, boolean resetEncoders){
        this.hm = hm;
        init(resetEncoders);
    }

    void init(boolean resetEncoders){
        lift1 = hm.get(DcMotorEx.class, LIFT1_NAME);
        lift2 = hm.get(DcMotorEx.class, LIFT2_NAME);
        if(resetEncoders)resetEncoders();

        if(reversed1) lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setPower(liftPower);
        lift1.setTargetPosition(downPosition);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setTargetPositionTolerance(20);
        MotorConfigurationType motorConfigurationType = lift1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lift1.setMotorType(motorConfigurationType);

        if(reversed2) lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setPower(liftPower);
        lift2.setTargetPosition(downPosition);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setTargetPositionTolerance(20);
//        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorConfigurationType = lift2.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lift2.setMotorType(motorConfigurationType);



        state = State.GOING_DOWN;
        nanoClock = NanoClock.system();
    }

    void resetEncoders(){
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double timeOfLastStateChange;

    void setState(State state){
        if(state == this.state) return;
        timeOfLastStateChange = nanoClock.seconds();
        this.state = state;
    }

    public static int liftTolerance = 16;

    private void updateState(){
        switch (state){
            case GOING_DOWN:
                if(Math.abs(lift1.getCurrentPosition() - state.pos) <= liftTolerance && Math.abs(lift2.getCurrentPosition() - state.pos) <= liftTolerance)
                    setState(State.DOWN);
                break;
            case GOING_LOW:
                if(Math.abs(lift1.getCurrentPosition() - state.pos) <= liftTolerance && Math.abs(lift2.getCurrentPosition() - state.pos) <= liftTolerance)
                    setState(State.LOW);
                break;
            case GOING_MID:
                if(Math.abs(lift1.getCurrentPosition() - state.pos) <= liftTolerance && Math.abs(lift2.getCurrentPosition() - state.pos) <= liftTolerance)
                    setState(State.MID);
                break;
            case GOING_HIGH:
                if(Math.abs(lift1.getCurrentPosition() - state.pos) <= liftTolerance && Math.abs(lift2.getCurrentPosition() - state.pos) <= liftTolerance)
                    setState(State.HIGH);
                break;
        }
    }

    void updateMotors(){
        target = state.pos + ground;

        pid.setPID(p,i,d);

        double power = pid.calculate(lift1.getCurrentPosition(), target);

//        lift1.setPower(f +power);
//        lift2.setPower(f + power);

        lift1.setTargetPosition(target);
        lift2.setTargetPosition(target);
//        lift2.setVelocity(lift1.getVelocity());
    }

    @Override
    public void atStart(){
        setState(State.GOING_DOWN);
    }

    @Override
    public void loop() {
        updateState();
        updateMotors();
    }
}
