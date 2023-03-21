package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Lift implements IRobotModule {

    public static boolean ENABLE_MODULE = true;


    public static String LIFT1_NAME = "lift1";
    public static String LIFT2_NAME = "lift2";
    public static boolean reversed1 = true, reversed2 = false;
    public static boolean resetEncoders;
    public int ground;

    HardwareMap hm;
    NanoClock nanoClock;

    DcMotorEx lift1, lift2;

    public static int downPosition = 0, lowPosition = 0, midPosition = 540, highPosition=932;
    public static double liftPower = 1;

    public enum State{
        DOWN(downPosition),
        LOW(lowPosition),
        MID(midPosition),
        HIGH(highPosition),
        GOING_DOWN(downPosition),
        GOING_LOW(lowPosition),
        GOING_MID(midPosition),
        GOING_HIGH(highPosition);

        int pos;
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
        lift1.setTargetPosition(lift1.getCurrentPosition());
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(reversed2) lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setPower(liftPower);
        lift2.setTargetPosition(lift2.getCurrentPosition());
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        state = State.GOING_DOWN;
        nanoClock = NanoClock.system();
    }

    void resetEncoders(){
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    double timeOfLastStateChange;

    void setState(State state){
        if(state == this.state) return;
        timeOfLastStateChange = nanoClock.seconds();
        this.state = state;
    }

    public static int liftTolerance = 16;

    private void updateState(){
        switch (state){
            case GOING_DOWN:
                if(Math.abs(lift1.getCurrentPosition() - downPosition) <= liftTolerance && Math.abs(lift2.getCurrentPosition() - downPosition) <= liftTolerance)
                    setState(State.DOWN);
                break;
            case GOING_LOW:
                if(Math.abs(lift1.getCurrentPosition() - lowPosition) <= liftTolerance && Math.abs(lift2.getCurrentPosition() - lowPosition) <= liftTolerance)
                    setState(State.LOW);
                break;
            case GOING_MID:
                if(Math.abs(lift1.getCurrentPosition() - midPosition) <= liftTolerance && Math.abs(lift2.getCurrentPosition() - midPosition) <= liftTolerance)
                    setState(State.MID);
                break;
            case GOING_HIGH:
                if(Math.abs(lift1.getCurrentPosition() - highPosition) <= liftTolerance && Math.abs(lift2.getCurrentPosition() - highPosition) <= liftTolerance)
                    setState(State.HIGH);
                break;
        }
    }

    void updateTargetPositions(){
        lift1.setTargetPosition(state.pos + ground);
        lift2.setTargetPosition(state.pos + ground);
    }

    @Override
    public void atStart(){
        setState(State.GOING_DOWN);
    }

    @Override
    public void loop() {
        updateState();
        updateTargetPositions();
    }
}
