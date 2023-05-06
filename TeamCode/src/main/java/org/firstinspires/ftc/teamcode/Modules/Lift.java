package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Lift implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public static String LIFT1_NAME = "lift1";
    public static String LIFT2_NAME = "lift2";
    public static boolean reversed1 = false, reversed2 = true;
    public int ground = 0;

    HardwareMap hm;
    NanoClock nanoClock;

    public DcMotorEx lift1, lift2;

    public static int downPosition = 0, lowPosition = downPosition, midPosition = 390, highPosition=660;
    public int target = downPosition;
    public static double liftPower = 1;

//    public static double p = 0,i = 0,d = 0;
//    public static double f = 0;

    public static PIDFCoefficients pidfCoefficients =  new PIDFCoefficients(0.01,0.28,0.0008,0);
    PIDFController pidf = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);

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

        static void update(){
            DOWN.pos = downPosition;
            LOW.pos = lowPosition;
            MID.pos = midPosition;
            HIGH.pos = highPosition;
            GOING_DOWN.pos = downPosition;
            GOING_LOW.pos = lowPosition;
            GOING_MID.pos = midPosition;
            GOING_HIGH.pos = highPosition;
        }
    }

    public State state, previousState;

    public Lift(HardwareMap hm, boolean resetEncoders){
        this.hm = hm;
        init(resetEncoders);
    }

    void init(boolean resetEncoders){
        lift1 = hm.get(DcMotorEx.class, LIFT1_NAME);
        lift2 = hm.get(DcMotorEx.class, LIFT2_NAME);
        if(resetEncoders)resetEncoders();

        if(reversed1) lift1.setDirection(DcMotorSimple.Direction.REVERSE);
//        lift1.setPower(liftPower);
//        lift1.setTargetPosition(downPosition);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lift1.setTargetPositionTolerance(2);
        MotorConfigurationType motorConfigurationType = lift1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lift1.setMotorType(motorConfigurationType);

        if(reversed2) lift2.setDirection(DcMotorSimple.Direction.REVERSE);
//        lift2.setPower(liftPower);
//        lift2.setTargetPosition(downPosition);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lift2.setTargetPositionTolerance(2);
        motorConfigurationType = lift2.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lift2.setMotorType(motorConfigurationType);

        state = State.GOING_DOWN;
        previousState = State.DOWN;
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
        this.previousState = this.state;
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

    public static double power;

    void updateMotors(){
        target = state.pos + ground;

        pidf.setPIDF(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f);

        power = pidf.calculate(lift1.getCurrentPosition(), target);

        lift1.setPower(power);
        lift2.setPower(power);
    }

    @Override
    public void atStart(){
        setState(State.GOING_DOWN);
    }

    @Override
    public void loop() {
        State.update();
        updateState();
        updateMotors();
    }
}
