package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Latch implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public static String LATCH_SERVO_NAME = "latch";
    public static boolean reversed = false;

    HardwareMap hm;
    NanoClock nanoClock;

    Servo latch;

    public static double openedlatchPosition = 0.25, closedlatchPosition = 0.67;
    public static double openingTime = 0.1, closingTime = 0.1, idlingTime = 0.1;

    public enum State{
        OPENED(openedlatchPosition),
        CLOSED(closedlatchPosition),
        MOPENED(openedlatchPosition),
        MCLOSED(closedlatchPosition),
        OPENING(openedlatchPosition),
        CLOSING(closedlatchPosition),
        MOPENING(openedlatchPosition),
        MCLOSING(closedlatchPosition);

        final double pos;
        State(double pos){
            this.pos = pos;
        }
    }

    public State state;

    public Latch(HardwareMap hm){
        this.hm = hm;
        init();
    }

    private void init(){
        latch = hm.get(Servo.class, LATCH_SERVO_NAME);
        if(reversed) latch.setDirection(Servo.Direction.REVERSE);
        nanoClock = NanoClock.system();
    }

    private double elapsedTime(double timeStamp){
        return nanoClock.seconds() - timeStamp;
    }

    double timeOfLastStateChange;

    public void setState(State state){
        if(state == this.state) return;
        timeOfLastStateChange = nanoClock.seconds();
        this.state = state;
    }

    private void updateState(){
        switch (state){
            case OPENING:
                if(elapsedTime(timeOfLastStateChange) >= openingTime) setState(State.OPENED);
                break;
            case MOPENING:
                if(elapsedTime(timeOfLastStateChange) >= openingTime) setState(State.MOPENED);
                break;
            case CLOSING:
                if(elapsedTime(timeOfLastStateChange) >= closingTime) setState(State.CLOSED);
                break;
            case MCLOSING:
                if(elapsedTime(timeOfLastStateChange) >= closingTime) setState(State.MCLOSED);
                break;
        }
    }

    private void updateTargetPosition(){
        latch.setPosition(state.pos);
    }

    @Override
    public void atStart(){
        setState(State.CLOSING);
    }

    @Override
    public void loop() {
        updateState();
        updateTargetPosition();
    }

}
