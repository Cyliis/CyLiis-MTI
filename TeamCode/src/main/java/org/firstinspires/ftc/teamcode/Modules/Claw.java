package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Claw implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public static String CLAW_SERVO_NAME = "claw";
    public static boolean reversed = false;

    HardwareMap hm;
    NanoClock nanoClock;

    Servo claw;

    public static double openedClawPosition = .53, closedClawPosition = .36;
    public static double openingTime = 0.2, closingTime = 0.15;

    public enum State{
        OPENED(openedClawPosition),
        CLOSED(closedClawPosition),
        MOPENED(openedClawPosition),
        MCLOSED(closedClawPosition),
        OPENING(openedClawPosition),
        CLOSING(closedClawPosition),
        MOPENING(openedClawPosition),
        MCLOSING(closedClawPosition);

        final double pos;
        State(double pos){
            this.pos = pos;
        }
    }

    public State state;

    public Claw(HardwareMap hm){
        this.hm = hm;
        init();
    }

    private void init(){
        claw = hm.get(Servo.class, CLAW_SERVO_NAME);
        if(reversed) claw.setDirection(Servo.Direction.REVERSE);
        nanoClock = NanoClock.system();
    }

    private double elapsedTime(double timeStamp){
        return nanoClock.seconds() - timeStamp;
    }

    double timeOfLastStateChange;

    public double debugCount = 0;

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
        claw.setPosition(state.pos);
    }

    @Override
    public void atStart(){
        setState(State.OPENED);
    }

    @Override
    public void loop() {
        debugCount = claw.getPosition();
        updateState();
        updateTargetPosition();
    }

}
