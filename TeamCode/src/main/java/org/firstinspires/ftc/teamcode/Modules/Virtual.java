package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.opencv.core.Mat;

@Config
public class Virtual implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public static String VIRTUAL1_SERVO_NAME = "virtual1";
    public static String VIRTUAL2_SERVO_NAME = "virtual2";
    public static boolean reversed1 = false, reversed2 = true;

    HardwareMap hm;
    NanoClock nanoClock;

    Servo virtual1, virtual2;

    public static double stack1 = 0.78, stack2 = 0.72, stack3 = 0.68, stack4 = 0.64, stack5 = 0.6;
    public static double[] stack = {stack1, stack2, stack3, stack4, stack5};
    public static int stackIndex = 0;
    public static double downPosition = stack[stackIndex], hoverPosition=0.74, lowPosition = 0.39, rotatePosition=0.21, transferPosition=0.13;
    public static double servoSpeed = 1;
    public double lastPos;
    public static double debugCounter;

    public enum State {
        DOWN(downPosition), HOVER(hoverPosition), LOW(lowPosition), ROTATE(rotatePosition), TRANSFER(transferPosition),
        GOING_DOWN(downPosition), GOING_HOVER(hoverPosition), GOING_LOW(lowPosition), GOING_ROTATE(rotatePosition), GOING_TRANSFER(transferPosition);

        public double pos;

        State(double pos) {
            this.pos = pos;
        }
    }

    public State state;

    public Virtual(HardwareMap hm) {
        this.hm = hm;
        init();
    }

    private void init() {
        nanoClock = NanoClock.system();
        virtual1 = hm.get(Servo.class, VIRTUAL1_SERVO_NAME);
        virtual2 = hm.get(Servo.class, VIRTUAL2_SERVO_NAME);
        if(reversed1) virtual1.setDirection(Servo.Direction.REVERSE);
        if(reversed2) virtual2.setDirection(Servo.Direction.REVERSE);
    }

    private double elapsedTime(double timeStamp) {
        return nanoClock.seconds() - timeStamp;
    }

    double timeOfLastStateChange;

    public void setState(State state) {
        if (state == this.state) return;
        debugCounter++;
        timeOfLastStateChange = nanoClock.seconds();
        this.state = state;
    }

    private void updateState() {
        switch (state) {
            case GOING_DOWN:
            case GOING_HOVER:
            case GOING_LOW:
            case GOING_ROTATE:
            case GOING_TRANSFER:
                if(elapsedTime(timeOfLastStateChange)*servoSpeed >= Math.abs(lastPos - state.pos))
                    switch (state){
                        case GOING_DOWN:
                            setState(State.DOWN);
                            break;
                        case GOING_HOVER:
                            setState(State.HOVER);
                            break;
                        case GOING_LOW:
                            setState(State.LOW);
                            break;
                        case GOING_ROTATE:
                            setState(State.ROTATE);
                            break;
                        case GOING_TRANSFER:
                            setState(State.TRANSFER);
                            break;
                    }
                break;
            case DOWN:
            case HOVER:
            case LOW:
            case ROTATE:
            case TRANSFER:
                lastPos = state.pos;
        }
    }

    private void updateTargetPosition() {
        virtual1.setPosition(state.pos);
        virtual2.setPosition(state.pos);
    }

    @Override
    public void atStart() {
        setState(State.DOWN);
    }

    @Override
    public void loop() {
        updateState();
        updateTargetPosition();
    }
}

