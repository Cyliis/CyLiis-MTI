package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Utils.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.Utils.DumbEncoder;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Lift implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public static String LIFT1_NAME = "lift1";
    public static String LIFT2_NAME = "lift2";
    public static boolean reversed1 = true, reversed2 =false, reversedEnc = true;
    public int ground = 0;

    HardwareMap hm;
    NanoClock nanoClock;

    public DcMotorEx lift1, lift2;
    public DumbEncoder liftEncoder;

    public static int downPosition = 0, lowPosition = downPosition, midPosition = 360, highPosition=590;
    public static int midPositionA = 360, highPositionA = 610;
    public static int lowerPosition = -30;
    public int target = downPosition;
    public static PIDCoefficients pidCoefficients =  new PIDCoefficients(0.01,0.12, 0.0008);
    public static PIDCoefficients telePid = new PIDCoefficients(0.01,0.12, 0.0008), autoPid = new PIDCoefficients(0.01,0.4,0.0006);
//    public static PIDCoefficients pidCoefficients =  new PIDCoefficients(0.01,0.3, 0.00045);
    public static double f1 = 0.12, f2 = 0.06;
//    public static double f1 = 0.1, f2 = 0.1;
    public static double maxPos = 630;
    PIDController pid = new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);

    public boolean correctPid = false;

    public static double maxVelocity = 1000000, acceleration = 8000, deceleration = 3000;

    public AsymmetricMotionProfile profile = new AsymmetricMotionProfile(maxVelocity, acceleration, deceleration);

    public enum State{
        DOWN(downPosition),
        LOW(lowPosition),
        MID(midPosition),
        HIGH(highPosition),
        GOING_DOWN(downPosition),
        GOING_LOW(lowPosition),
        GOING_MID(midPosition),
        GOING_HIGH(highPosition),
        RESETTING(lowerPosition);

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

        public static void auto(){
            MID.pos = midPositionA;
            GOING_MID.pos = midPositionA;
            HIGH.pos = highPositionA;
            GOING_HIGH.pos = highPositionA;
        }

        public static void tele(){
            MID.pos = midPosition;
            GOING_MID.pos = midPosition;
            HIGH.pos = highPosition;
            GOING_HIGH.pos = highPosition;
        }
    }

    public State state, previousState;

    public Lift(HardwareMap hm, boolean resetEncoders){
        this.hm = hm;
        correctPid = false;
        init(resetEncoders);
    }

    void init(boolean resetEncoders){
        lift1 = hm.get(DcMotorEx.class, LIFT1_NAME);
        lift2 = hm.get(DcMotorEx.class, LIFT2_NAME);
        if(resetEncoders)resetEncoders();

        if(reversed1) lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorConfigurationType motorConfigurationType = lift1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lift1.setMotorType(motorConfigurationType);

        if(reversed2) lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorConfigurationType = lift2.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lift2.setMotorType(motorConfigurationType);

        liftEncoder = new DumbEncoder(hm,"mfr", reversedEnc);

        state = State.GOING_DOWN;
        previousState = State.DOWN;
        nanoClock = NanoClock.system();
    }

    public void setCorrectPid(boolean auto){
        if(correctPid) return;
        if(auto) pidCoefficients = autoPid;
        else pidCoefficients = telePid;
        correctPid = true;
    }

    void resetEncoders(){
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double timeOfLastStateChange;

    private boolean looped = false;

    void setState(State state){
        if(state == this.state) return;
        timeOfLastStateChange = nanoClock.seconds();
        this.previousState = this.state;
        this.state = state;
        double vel = profile.getSignedVelocity();
        double pos = profile.getPosition();

        profile = new AsymmetricMotionProfile(maxVelocity, acceleration, deceleration);
        profile.setMotion(pos, state.pos, vel);
        if(state == State.RESETTING){
            looped = false;
        }
    }

    public static int liftTolerance = 40;

    private void updateState(){
        switch (state){
            case GOING_DOWN:
                if(Math.abs(liftEncoder.getCurrentPosition() - state.pos) <= liftTolerance)
                    setState(State.RESETTING);
                break;
            case GOING_LOW:
                if(Math.abs(liftEncoder.getCurrentPosition() - state.pos) <= liftTolerance)
                    setState(State.LOW);
                break;
            case GOING_MID:
                if(Math.abs(liftEncoder.getCurrentPosition() - state.pos) <= liftTolerance)
                    setState(State.MID);
                break;
            case GOING_HIGH:
                if(Math.abs(liftEncoder.getCurrentPosition() - state.pos) <= liftTolerance)
                    setState(State.HIGH);
                break;
            case RESETTING:
                if(looped){
                    if(liftEncoder.getVelocity() == 0){
                        ground = liftEncoder.getCurrentPosition();
                        setState(State.DOWN);
                    }
                }
                else looped = true;
                break;
        }
    }

    public static double power;

    void updateMotors(){
        target = state.pos + ground;
        int current = liftEncoder.getCurrentPosition();

        pid.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);

        power = pid.calculate(liftEncoder.getCurrentPosition(), target) + f1*((double)current/(double)maxPos) + f2;

        lift1.setPower(power);
        lift2.setPower(power);
    }

    @Override
    public void atStart(){
        setState(State.GOING_DOWN);
    }

    @Override
    public void loop() {
        profile.update();
        State.update();
        updateState();
        updateMotors();
    }

    @Override
    public void emergencyStop(){
        lift1.setPower(0);
        lift2.setPower(0);
    }
}
