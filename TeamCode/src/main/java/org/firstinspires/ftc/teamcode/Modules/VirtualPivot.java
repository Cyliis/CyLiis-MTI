package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class VirtualPivot implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public static String VIRTUAL_PIVOT_SERVO_NAME = "pivot";
    public static boolean reversed = false;

    HardwareMap hm;
    NanoClock nanoClock;

    Servo pivot;

    public static double frontPivotPosition = 0.20, backPivotPosition = .87;
    public static double frontRotationTime = 0.1, backRotationTime = 0.1;

    public static int debugCounter = 0;

    public enum State{
        FRONT(frontPivotPosition), BACK(backPivotPosition),
        RFRONT(frontPivotPosition), RBACK(backPivotPosition);

        final double pos;
        State(double pos){
            this.pos = pos;
        }
    }

    public State state;

    public VirtualPivot(HardwareMap hm){
        this.hm = hm;
        init();
    }

    private void init(){
        pivot = hm.get(Servo.class, VIRTUAL_PIVOT_SERVO_NAME);
        if(reversed) pivot.setDirection(Servo.Direction.REVERSE);
        nanoClock = NanoClock.system();
    }

    private double elapsedTime(double timeStamp){
        return nanoClock.seconds() - timeStamp;
    }

    double timeOfLastStateChange;

    public void setState(State state){
        debugCounter++;
        if(state == this.state) return;
        timeOfLastStateChange = nanoClock.seconds();
        this.state = state;
    }

    private void updateState(){
        switch (state){
            case RFRONT:
                if(elapsedTime(timeOfLastStateChange) >= frontRotationTime) setState(State.FRONT);
                break;
            case RBACK:
                if(elapsedTime(timeOfLastStateChange) >= backRotationTime) setState(State.BACK);
                break;
        }
    }

    private void updateTargetPosition(){
        pivot.setPosition(state.pos);
    }

    @Override
    public void atStart(){
        setState(State.FRONT);
    }

    @Override
    public void loop() {
        updateState();
        updateTargetPosition();
    }
}
