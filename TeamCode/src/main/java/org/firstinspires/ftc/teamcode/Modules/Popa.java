package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Popa implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public static String POPA_SERVO_NAME = "funny";
    public static boolean reversed = false;

    HardwareMap hm;
    NanoClock nanoClock;

    Servo popa;

    public static double downPopaPosition = 1, upPopaPosition = .5;
    public static double downTime = 0.2, upTime = 0.5;

    public enum State{
        GOING_DOWN(downPopaPosition), DOWN(downPopaPosition),
        GOING_UP(upPopaPosition), UP(upPopaPosition);

        final double pos;
        State(double pos){
            this.pos = pos;
        }
    }

    public State state;

    public Popa(HardwareMap hm){
        this.hm = hm;
        init();
    }

    private void init(){
        popa = hm.get(Servo.class, POPA_SERVO_NAME);
        if(reversed) popa.setDirection(Servo.Direction.REVERSE);
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
            case GOING_UP:
                if(elapsedTime(timeOfLastStateChange) >= upTime) setState(State.UP);
                break;
            case GOING_DOWN:
                if(elapsedTime(timeOfLastStateChange) >= downTime) setState(State.DOWN);
                break;
        }
    }

    private void updateTargetPosition(){
        popa.setPosition(state.pos);
    }

    @Override
    public void atStart(){
        setState(State.UP);
    }

    @Override
    public void loop() {
        updateState();
        updateTargetPosition();
    }
}
