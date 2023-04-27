package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class UtaUta implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public static String UTAUTA_SERVO_NAME = "uta";
    public static boolean reversed = false;

    HardwareMap hm;
    NanoClock nanoClock;

    Servo uta;

    public static double levelPosition = 0.5, angledPosition = 0.66, hoverPosition = 0.5;
    public static double levelingTime = 0.25, anglingTime = 0.25;

    public enum State{
        LEVEL(levelPosition), ANGLED(angledPosition),
        LEVELING(levelPosition), ANGLING(angledPosition);

        public double pos;
        State(double pos){
            this.pos = pos;
        }
    }

    public State state;

    public UtaUta(HardwareMap hm){
        this.hm = hm;
        init();
    }

    private void init(){
        uta = hm.get(Servo.class, UTAUTA_SERVO_NAME);
        if(reversed) uta.setDirection(Servo.Direction.REVERSE);
        state = State.LEVELING;
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
            case LEVELING:
                if(elapsedTime(timeOfLastStateChange) >= levelingTime) setState(State.LEVEL);
                break;
            case ANGLING:
                if(elapsedTime(timeOfLastStateChange) >= anglingTime) setState(State.ANGLED);
                break;
        }
    }

    private void updateTargetPosition(){
        uta.setPosition(state.pos);
    }

    @Override
    public void atStart(){
        setState(State.LEVELING);
    }

    @Override
    public void loop() {
        updateState();
        updateTargetPosition();
    }
}
