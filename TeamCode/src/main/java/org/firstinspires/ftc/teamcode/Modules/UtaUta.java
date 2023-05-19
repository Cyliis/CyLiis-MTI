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

    public static double levelPosition = 0.585, angledPosition = 0.735, bruhPosition = 0.61, frontBruhPosition = 0.51;
    public static double levelingTime = 0.05, anglingTime = 0.05, smolAnglingTime = 0.15;

    public enum State{
        LEVEL(levelPosition), ANGLED(angledPosition), SMOLANGLE(bruhPosition), SMOLANGLEFRONT(frontBruhPosition),
        LEVELING(levelPosition), ANGLING(angledPosition), SMOLANGELING(bruhPosition), SMOLAGNLINGFRONT(frontBruhPosition);

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
            case SMOLANGELING:
                if(elapsedTime(timeOfLastStateChange) >= smolAnglingTime) setState(State.SMOLANGLE);
                break;
            case SMOLAGNLINGFRONT:
                if(elapsedTime(timeOfLastStateChange) >= smolAnglingTime) setState(State.SMOLANGLEFRONT);
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
