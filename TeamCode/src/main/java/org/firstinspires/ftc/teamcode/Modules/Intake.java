package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Intake implements IRobotModule {
    public static boolean ENABLE_MODULE = true;

    NanoClock nanoClock;

    public Claw claw;
    public Virtual virtual;
    public Latch latch;
    public VirtualPivot pivot;
    public UtaUta uta;

    public Intake(HardwareMap hm, Latch latch, Virtual virtual, Claw claw){
        this.claw = claw;
        this.latch = latch;
        this.virtual = virtual;
        this.uta = new UtaUta(hm);
        pivot = new VirtualPivot(hm);

        state = State.OPENING;

        nanoClock = NanoClock.system();
    }

    public enum State{
        OPENED, CLOSED, MOPENED, MCLOSED, HOVERING, MHOVERING,
        OPENING, CLOSING, MOPENING, MCLOSING, TRANSFERING,
        GOING_LOW, LOW, REALEASING_LOW, RELEASED_LOW, GOING_DOWN_FROM_LOW;
    }

    public State state;

    double timeOfLastStateChange;

    public void setState(State state){
        if(state==this.state) return;
        this.state = state;
        switch (state){
            case TRANSFERING:
                transferState = TransferState.START;
                break;
            case OPENING:
                if(uta.state!= UtaUta.State.LEVEL && uta.state != UtaUta.State.LEVELING) uta.setState(UtaUta.State.LEVELING);
                virtual.setState(Virtual.State.GOING_DOWN);
                claw.setState(Claw.State.OPENING);
                break;
            case MOPENING:
                if(uta.state!= UtaUta.State.LEVEL && uta.state != UtaUta.State.LEVELING) uta.setState(UtaUta.State.LEVELING);
                virtual.setState(Virtual.State.GOING_DOWN);
                claw.setState(Claw.State.MOPENING);
                break;
            case CLOSING:
                if(uta.state!= UtaUta.State.LEVEL && uta.state != UtaUta.State.LEVELING) uta.setState(UtaUta.State.LEVELING);
                claw.setState(Claw.State.CLOSING);
                break;
            case MCLOSING:
                if(uta.state!= UtaUta.State.LEVEL && uta.state != UtaUta.State.LEVELING) uta.setState(UtaUta.State.LEVELING);
                claw.setState(Claw.State.MCLOSING);
                break;
            case HOVERING:
            case MHOVERING:
                if(uta.state!= UtaUta.State.LEVEL && uta.state != UtaUta.State.LEVELING) uta.setState(UtaUta.State.LEVELING);
                virtual.setState(Virtual.State.GOING_HOVER);
                break;
            case GOING_LOW:
                virtual.setState(Virtual.State.GOING_LOW);
                uta.setState(UtaUta.State.ANGLING);
                break;
            case REALEASING_LOW:
                claw.setState(Claw.State.MOPENING);
                break;
            case GOING_DOWN_FROM_LOW:
                if(uta.state!= UtaUta.State.LEVEL && uta.state != UtaUta.State.LEVELING) uta.setState(UtaUta.State.LEVELING);
                virtual.setState(Virtual.State.GOING_DOWN);
                claw.setState(Claw.State.MOPENING);
                break;
        }
        timeOfLastStateChange = nanoClock.seconds();
    }

    void updateState(){
        switch (state){
            case OPENING:
                if(uta.state == UtaUta.State.LEVEL && claw.state == Claw.State.OPENED) setState(State.OPENED);
                break;
            case MOPENING:
                if(uta.state == UtaUta.State.LEVEL && claw.state == Claw.State.MOPENED) setState(State.MOPENED);
                break;
            case CLOSING:
                if(uta.state == UtaUta.State.LEVEL && claw.state == Claw.State.CLOSED) {
                    setState(State.CLOSED);
                }
                break;
            case MCLOSING:
                if(uta.state == UtaUta.State.LEVEL && claw.state == Claw.State.MCLOSED)  {
                    setState(State.MCLOSED);
                }
                break;
            case TRANSFERING:
                if(transferState == TransferState.END) setState(State.OPENED);
                break;
            case HOVERING:
                if(uta.state == UtaUta.State.LEVEL && virtual.state == Virtual.State.HOVER) setState(State.CLOSED);
                break;
            case MHOVERING:
                if(uta.state == UtaUta.State.LEVEL && virtual.state == Virtual.State.HOVER) setState(State.MCLOSED);
                break;
            case GOING_LOW:
                if(virtual.state == Virtual.State.LOW && uta.state == UtaUta.State.ANGLED)
                    setState(State.LOW);
                break;
            case REALEASING_LOW:
                if(claw.state == Claw.State.MOPENED){
                    setState(State.RELEASED_LOW);
                }
                break;
            case GOING_DOWN_FROM_LOW:
                if(virtual.state == Virtual.State.DOWN && claw.state == Claw.State.MOPENED){
                    claw.setState(Claw.State.OPENED);
                    setState(State.OPENED);
                }
                break;
        }
    }

    public enum TransferState{
        START,
        CLOSE_CLAW,
        OPEN_LATCH_AND_LIFT_VIRTUAL_TO_ROTATE_BACK,
        ROTATE_BACK,
        VIRTUAL_TO_TRANSFER,
        CLOSE_LATCH,
        OPEN_CLAW_TRANSFER,
        VIRTUAL_TO_ROTATE_FRONT,
        CLOSE_CLAW_TRANSFER,
        ROTATE_FRONT,
        VIRTUAL_DOWN,
        OPEN_CLAW,
        END
    }

    public TransferState transferState;

    void updateTransferState(){
        if(state!=State.TRANSFERING) transferState = TransferState.END;
        if(transferState == TransferState.END) return;
        switch (transferState){
            case START:
                if(claw.state!= Claw.State.CLOSED && claw.state!= Claw.State.MCLOSED)claw.setState(Claw.State.MCLOSING);
                transferState = TransferState.CLOSE_CLAW;
                break;
            case CLOSE_CLAW:
                if(claw.state == Claw.State.CLOSED || claw.state == Claw.State.MCLOSED){
                    latch.setState(Latch.State.OPENING);
                    virtual.setState(Virtual.State.GOING_ROTATE);
                    transferState = TransferState.OPEN_LATCH_AND_LIFT_VIRTUAL_TO_ROTATE_BACK;
                }
                break;
            case OPEN_LATCH_AND_LIFT_VIRTUAL_TO_ROTATE_BACK:
                if(latch.state == Latch.State.OPENED && virtual.state== Virtual.State.ROTATE){
                    pivot.setState(VirtualPivot.State.RBACK);
                    transferState = TransferState.ROTATE_BACK;
                }
                break;
            case ROTATE_BACK:
                if(pivot.state == VirtualPivot.State.BACK){
                    virtual.setState(Virtual.State.GOING_TRANSFER);
                    transferState = TransferState.VIRTUAL_TO_TRANSFER;
                }
                break;
            case VIRTUAL_TO_TRANSFER:
                if(virtual.state == Virtual.State.TRANSFER){
                    latch.setState(Latch.State.CLOSING);
                    transferState = TransferState.CLOSE_LATCH;
                }
                break;
            case CLOSE_LATCH:
                if(latch.state == Latch.State.CLOSED){
                    claw.setState(Claw.State.MOPENING);
                    transferState = TransferState.OPEN_CLAW_TRANSFER;
                }
                break;
            case OPEN_CLAW_TRANSFER:
                if(claw.state == Claw.State.MOPENED){
                    virtual.setState(Virtual.State.GOING_ROTATE);
                    transferState = TransferState.VIRTUAL_TO_ROTATE_FRONT;
                }
                break;
            case VIRTUAL_TO_ROTATE_FRONT:
                if(virtual.state == Virtual.State.ROTATE){
                    claw.setState(Claw.State.MCLOSING);
                    transferState = TransferState.CLOSE_CLAW_TRANSFER;
                }
                break;
            case CLOSE_CLAW_TRANSFER:
                if(claw.state == Claw.State.MCLOSED){
                    pivot.setState(VirtualPivot.State.RFRONT);
                    transferState = TransferState.ROTATE_FRONT;
                }
                break;
            case ROTATE_FRONT:
                if(pivot.state == VirtualPivot.State.FRONT){
                    virtual.setState(Virtual.State.GOING_DOWN);
                    transferState = TransferState.VIRTUAL_DOWN;
                }
            case VIRTUAL_DOWN:
                if(virtual.state == Virtual.State.DOWN){
                    claw.setState(Claw.State.MOPENING);
                    transferState = TransferState.OPEN_CLAW;
                }
                break;
            case OPEN_CLAW:
                if(claw.state == Claw.State.MOPENED){
                    claw.state = Claw.State.OPENED;
                    transferState = TransferState.END;
                }
        }
    }

    @Override
    public void atStart() {
        pivot.atStart();
        uta.loop();
    }

    @Override
    public void loop() {
        updateState();
        updateTransferState();
        pivot.loop();
        uta.loop();
    }
}
