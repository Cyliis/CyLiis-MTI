package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Outtake implements IRobotModule {
    public static boolean ENABLE_MODULE = true;

    NanoClock nanoClock;

    public Lift lift;
    public Virtual virtual;
    public Latch latch;
    public Claw claw;

    public Outtake(HardwareMap hm, Latch latch, Virtual virtual, Claw claw){
        this.lift = new Lift(hm);
        this.latch = latch;
        this.virtual = virtual;
        this.claw = claw;
        state = State.GOING_DOWN;
        nanoClock = NanoClock.system();
    }

    public enum State{
        DOWN, MID, HIGH,
        GOING_DOWN, GOING_DOWN_NO_LATCH, GOING_MID, GOING_HIGH;
    }

    public State state;

    double timeOfLastStateChange;

    public void setState(State state){
        if(state==this.state) return;
        switch (state){
            case DOWN:
            case GOING_DOWN:
                latch.setState(Latch.State.OPENING);
                lift.setState(Lift.State.GOING_DOWN);
                virtual.setState(Virtual.State.GOING_DOWN);
                break;
            case GOING_DOWN_NO_LATCH:
                virtual.setState(Virtual.State.GOING_DOWN);
                lift.setState(Lift.State.GOING_DOWN);
                break;
            case MID:
            case GOING_MID:
                lift.setState(Lift.State.GOING_MID);
                break;
            case HIGH:
            case GOING_HIGH:
                lift.setState(Lift.State.GOING_HIGH);
                break;
        }
        this.state = state;
        timeOfLastStateChange = nanoClock.seconds();
    }

    void updateState(){
        switch (state){
            case GOING_DOWN:
                if(lift.state == Lift.State.DOWN && virtual.state == Virtual.State.DOWN)
                    state = State.DOWN;
                break;
            case GOING_DOWN_NO_LATCH:
                if(virtual.state == Virtual.State.DOWN && lift.state == Lift.State.DOWN){
                    state = State.DOWN;
                }
                break;
            case GOING_MID:
                if(lift.state == Lift.State.MID)
                    state = State.MID;
                break;
            case GOING_HIGH:
                if(lift.state == Lift.State.HIGH)
                    state = State.HIGH;
                break;
        }
    }

    @Override
    public void atStart() {
        lift.atStart();
    }

    @Override
    public void loop() {
        updateState();
        lift.loop();
    }
}
