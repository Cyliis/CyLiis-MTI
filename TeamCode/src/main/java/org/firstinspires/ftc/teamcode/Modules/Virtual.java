package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Virtual implements IRobotModule {

    public static boolean ENABLE_MODULE = true;


    public static String VIRTUAL_LEFT_NAME = "virtual1";
    public static String VIRTUAL_RIGHT_NAME = "virtual2";
    public static String VIRTUAL_ENCODER_NAME = "virtualEncoder";
    public static boolean reversed1 = false , reversed2 = true, reversedEnc = true;
    public static double TICKS_PER_REV = 8192;

    HardwareMap hm;
    NanoClock nanoClock;

    Servo virtual1 , virtual2;
    public DcMotorEx virtualEncoder;

    public static double stack1 = 0.22, stack2 = 0.28, stack3 = 0.32, stack4 = 0.355, stack5 = 0.4;
    public static double stack1E = 80, stack2E = 380, stack3E = 584, stack4E = 860, stack5E = 1087;
    public static double[] stack = {stack1, stack2, stack3, stack4, stack5};
    public static double[] stackE = {stack1E, stack2E, stack3E, stack4E, stack5E};
    public static int stackIndex = 0;
    public static double downPosition = stack[stackIndex], hoverPosition = 0.295, lowPosition = 0.58, transferPosition = 0.87;
    public static double downPositionE = stackE[stackIndex], hoverPositionE = 460, lowPositionE = 2150, transferPositionE = 4150, rotatePositionFromFrontE = 1600, rotatePositionFromBackE = 3000;

    public enum State{
        GOING_DOWN(downPosition, downPositionE),
        DOWN(downPosition, downPositionE),
        GOING_HOVER(hoverPosition, hoverPositionE),
        HOVER(hoverPosition, hoverPositionE),
        GOING_LOW(lowPosition, lowPositionE),
        LOW(lowPosition, lowPositionE),
        GOING_TRANSFER(transferPosition, transferPositionE),
        TRANSFER(transferPosition, transferPositionE);

        public double pos, encPos;
        State(double pos, double encPos){
            this.pos = pos;
            this.encPos = encPos;
        }
    }

    public State state;

    public Virtual(HardwareMap hm, boolean resetEncoders){
        this.hm = hm;
        init(resetEncoders);
    }

    void init(boolean resetEncoders){
        virtual1 = hm.get(Servo.class, VIRTUAL_LEFT_NAME);
        if(reversed1) virtual1.setDirection(Servo.Direction.REVERSE);
        virtual2 = hm.get(Servo.class, VIRTUAL_RIGHT_NAME);
        if(reversed2) virtual2.setDirection(Servo.Direction.REVERSE);

        virtualEncoder = hm.get(DcMotorEx.class, VIRTUAL_ENCODER_NAME);
        if(resetEncoders) resetEncoders();
        if(reversedEnc) virtualEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        state = State.GOING_DOWN;
        nanoClock = NanoClock.system();
    }

    void resetEncoders(){
        virtualEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    double timeOfLastStateChange;

    void setState(State state){
        if(state == this.state) return;
        timeOfLastStateChange = nanoClock.seconds();
        this.state = state;
    }

    public static int virtualTolerance = 200;

    private void updateState(){
        switch (state){
            case GOING_DOWN:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.encPos) <= virtualTolerance)
                    setState(State.DOWN);
                break;
            case GOING_HOVER:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.encPos) <= virtualTolerance)
                    setState(State.HOVER);
                break;
            case GOING_LOW:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.encPos) <= virtualTolerance)
                    setState(State.LOW);
                break;
            case GOING_TRANSFER:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.encPos) <= virtualTolerance)
                    setState(State.TRANSFER);
                break;
        }
    }

    void updateServos(){
        virtual1.setPosition(state.pos);
        virtual2.setPosition(state.pos);
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
