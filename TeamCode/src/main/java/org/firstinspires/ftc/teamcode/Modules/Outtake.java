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

    public static int latchReleasePointDiff = 10;

    public Outtake(HardwareMap hm, Latch latch, Virtual virtual, Claw claw, boolean resetEncoders){
        this.lift = new Lift(hm, resetEncoders);
        this.latch = latch;
        this.virtual = virtual;
        this.claw = claw;
        state = State.GOING_DOWN_NO_LATCH;
        nanoClock = NanoClock.system();
    }

    public enum State{
        DOWN, MID, HIGH,
        GOING_DOWN, GOING_DOWN_NO_LATCH, GOING_MID, GOING_HIGH;
    }

    public State state;

    public double timeOfLastStateChange;

    public void setState(State state){
        if(state==this.state) return;
        switch (state){
            case DOWN:
            case GOING_DOWN:
            case GOING_DOWN_NO_LATCH:
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
                if(latch.state == Latch.State.CLOSED || latch.state == Latch.State.CLOSING){
                    if(lift.liftEncoder.getCurrentPosition() <= lift.previousState.pos - latchReleasePointDiff)
                        latch.setState(Latch.State.OPENING);
                }
                if(lift.state == Lift.State.DOWN)
                    state = State.DOWN;
                break;
            case GOING_DOWN_NO_LATCH:
                if(lift.state == Lift.State.DOWN){
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
