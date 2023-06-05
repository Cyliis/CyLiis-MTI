package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.DumbEncoder;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Virtual implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public static String VIRTUAL_LEFT_NAME = "virtual1";
    public static String VIRTUAL_RIGHT_NAME = "virtual2";
    public static String VIRTUAL_ENCODER_NAME = "lift1";
    public static boolean reversed1 = true , reversed2 = false, reversedEnc = false;
    public static double TICKS_PER_REV = 8192;
    public static double ticksOffset = -1076;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.00125,0.065,0.00006,0.14);
    private final PIDController pid = new PIDController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);

    HardwareMap hm;
    NanoClock nanoClock;

    DcMotorEx virtual1 , virtual2;
    public DumbEncoder virtualEncoder;

    public static int stackIndex = 0;

    public static double stack1_s = 0.288, stack2_s = 0.33, stack3_s = 0.362, stack4_s = 0.406, stack5_s = 0.443;
    public static double[] stack_s = {stack1_s, stack2_s, stack3_s, stack4_s, stack5_s};
    public static double downPosition_s = stack_s[stackIndex], hoverPosition_s = 0.334, hoverPositionStack_s = 0.539, lowPosition_s = 0.862, transferPosition_s = 0.828, popaPosition_s = 0.7;
    public static double manualAdd_s = 0, manualMultiplier_s;

    public static double stack1 = 34, stack2 = 400, stack3 = 600, stack4 = 900, stack5 = 1080;
    public static double[] stack = {stack1, stack2, stack3, stack4, stack5};
    public static double downPosition = stack[stackIndex], hoverPosition = 400, hoverPositionStack = 1850, lowPosition = 4500, transferPosition = 4100, popaPosition = 3150;
    public static double rotatePositionFromFront = 0, rotatePositionFromBack = 5000, lowRotateFromFrontPosition = 0;
    public static double manualAdd = 0, manualMultiplier = 1200;

    public enum State{
        GOING_DOWN(downPosition, downPosition_s),
        DOWN(downPosition, downPosition_s),
        GOING_HOVER(hoverPosition, hoverPosition_s),
        HOVER(hoverPosition, hoverPosition_s),
        GOING_LOW(lowPosition, lowPosition_s),
        LOW(lowPosition, lowPosition_s),
        GOING_TRANSFER(transferPosition, transferPosition_s),
        TRANSFER(transferPosition, transferPosition_s),
        GOING_POPA(popaPosition, popaPosition_s),
        POPA(popaPosition, popaPosition_s);

        public double pos, pos_s;
        State(double pos, double pos_s){
            this.pos = pos;
            this.pos_s = pos_s;
        }
    }

    public State state;

    public Virtual(HardwareMap hm, boolean resetEncoders){
        this.hm = hm;
        init(resetEncoders);
    }

    void init(boolean resetEncoders){
        virtual1 = hm.get(DcMotorEx.class, VIRTUAL_LEFT_NAME);
        if(reversed1) virtual1.setDirection(DcMotorEx.Direction.REVERSE);
        virtual1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        virtual2 = hm.get(DcMotorEx.class, VIRTUAL_RIGHT_NAME);
        if(reversed2) virtual2.setDirection(DcMotorEx.Direction.REVERSE);
        virtual2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        virtualEncoder = new DumbEncoder(hm, VIRTUAL_ENCODER_NAME);
        if(resetEncoders) resetEncoders();
        if(reversedEnc) virtualEncoder.setReversed(true);
        state = State.DOWN;
        nanoClock = NanoClock.system();
    }

    void resetEncoders(){
        virtualEncoder.reset();
    }

    double timeOfLastStateChange;

    public void setState(State state){
        if(state == this.state) return;
        timeOfLastStateChange = nanoClock.seconds();

        this.state = state;
    }

    public static int virtualTolerance = 200;

    private void updateState(){
        switch (state){
            case GOING_DOWN:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.pos) <= virtualTolerance)
                    setState(State.DOWN);
                break;
            case GOING_HOVER:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.pos) <= virtualTolerance)
                    setState(State.HOVER);
                break;
            case GOING_LOW:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.pos) <= virtualTolerance)
                    setState(State.LOW);
                break;
            case GOING_TRANSFER:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.pos) <= virtualTolerance)
                    setState(State.TRANSFER);
                break;
            case GOING_POPA:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.pos) <= virtualTolerance)
                    setState(State.POPA);
                break;
        }
    }

    public double ff, power;

    public static double speedLimit = 0.75;

    void updateServos(){
        //TODO: PID

        ff = Math.cos(((double)virtualEncoder.getCurrentPosition() + ticksOffset) / TICKS_PER_REV * (Math.PI * 2)) * pidfCoefficients.f;
        pid.setPID(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);
        power = pid.calculate(virtualEncoder.getCurrentPosition() + manualAdd, state.pos);

        power = Math.min(speedLimit, power);
        power = Math.max(-speedLimit, power);

        virtual1.setPower(ff+power);
        virtual2.setPower(ff+power);

//        virtual1.setPosition(state.pos_s);
//        virtual2.setPosition(state.pos_s);
    }

    @Override
    public void atStart(){
        setState(State.GOING_DOWN);
    }

    @Override
    public void loop() {
        updateState();
        updateServos();
    }
}
