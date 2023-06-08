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
    public Popa popa;

    public Intake(HardwareMap hm, Latch latch, Virtual virtual, Claw claw){
        this.claw = claw;
        this.latch = latch;
        this.virtual = virtual;
        this.uta = new UtaUta(hm);
        pivot = new VirtualPivot(hm);
        popa = new Popa(hm);

        state = State.OPENING;

        nanoClock = NanoClock.system();
    }

    public enum State{
        OPENED, CLOSED, MOPENED, MCLOSED, HOVERING, MHOVERING,
        OPENING, CLOSING, MOPENING, MCLOSING, TRANSFERING,
        GOING_LOW, LOW, REALEASING_LOW, RELEASED_LOW, GOING_DOWN_FROM_LOW,
        GOING_UP_FOR_POPA, POPA_GOING_DOWN, POPA, POPA_GOING_UP, GOING_DOWN_FROM_POPA;
    }

    public State state;

    private State beforePopaState;

    public double timeOfLastStateChange;

    public void setState(State state){
        if(state==this.state) return;
        switch (state){
            case TRANSFERING:
                transferState = TransferState.START;
                break;
            case OPENING:
                if(uta.state!= UtaUta.State.LEVEL && uta.state != UtaUta.State.LEVELING) uta.setState(UtaUta.State.LEVELING);
                virtual.setState(Virtual.State.GOING_DOWN);
                claw.setState(Claw.State.OPENING);
                break;
            case GOING_DOWN_FROM_LOW:
                uta.setState(UtaUta.State.LEVELING);
                virtual.setState(Virtual.State.GOING_DOWN);
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
                uta.setState(UtaUta.State.SMOLAGNLINGFRONT);
                virtual.setState(Virtual.State.GOING_HOVER);
                break;
            case GOING_LOW:
                virtual.setState(Virtual.State.GOING_LOW);
                uta.setState(UtaUta.State.ANGLING);
                break;
            case LOW:
                rotatedLow = false;
                break;
            case REALEASING_LOW:
                claw.setState(Claw.State.MOPENING);
                Virtual.stackIndex = Math.max(Virtual.stackIndex - 1, 0);
                break;
            case GOING_UP_FOR_POPA:
                beforePopaState = this.state;
                virtual.setState(Virtual.State.GOING_POPA);
                break;
            case POPA_GOING_DOWN:
                popa.setState(Popa.State.GOING_DOWN);
                break;
            case POPA_GOING_UP:
                popa.setState(Popa.State.GOING_UP);
                break;
            case GOING_DOWN_FROM_POPA:
                virtual.setState(Virtual.State.GOING_DOWN);
        }
        this.state = state;
        timeOfLastStateChange = nanoClock.seconds();
    }

    boolean rotatedLow = false;

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
                if(uta.state == UtaUta.State.SMOLANGLEFRONT && virtual.state == Virtual.State.HOVER) setState(State.CLOSED);
                break;
            case MHOVERING:
                if(uta.state == UtaUta.State.SMOLANGLEFRONT && virtual.state == Virtual.State.HOVER) setState(State.MCLOSED);
                break;
            case GOING_LOW:
                if(virtual.virtualEncoder.getCurrentPosition() >= Virtual.lowRotateFromFrontPosition && !rotatedLow){
                    rotatedLow = true;
                    pivot.setState(VirtualPivot.State.RBACK);
                }
                if(virtual.state == Virtual.State.LOW && uta.state == UtaUta.State.ANGLED && pivot.state == VirtualPivot.State.BACK)
                    setState(State.LOW);
                break;
            case REALEASING_LOW:
                if(claw.state == Claw.State.MOPENED){
                    setState(State.GOING_DOWN_FROM_LOW);
                }
                break;
            case GOING_DOWN_FROM_LOW:
                if(pivot.state == VirtualPivot.State.BACK && virtual.virtualEncoder.getCurrentPosition() <= Virtual.rotatePositionFromBack){
                    pivot.setState(VirtualPivot.State.RFRONT);
                }
                if(virtual.state == Virtual.State.DOWN && pivot.state == VirtualPivot.State.FRONT && uta.state == UtaUta.State.LEVEL){
                    switch (claw.state){
                        case CLOSED:
                        case MCLOSED:
                            setState(State.CLOSED);
                            break;
                        case OPENED:
                        case MOPENED:
                            setState(State.OPENED);
                            break;
                    }
                }
                break;
            case GOING_UP_FOR_POPA:
                if(virtual.state == Virtual.State.POPA){
                    setState(State.POPA_GOING_DOWN);
                }
                break;
            case POPA_GOING_DOWN:
                if(popa.state == Popa.State.DOWN){
                    setState(State.POPA);
                }
                break;
            case POPA_GOING_UP:
                if(popa.state == Popa.State.UP){
                    setState(State.GOING_DOWN_FROM_POPA);
                }
                break;
            case GOING_DOWN_FROM_POPA:
                if(virtual.state == Virtual.State.DOWN){
                    setState(beforePopaState);
                }
                break;
        }
    }

    public enum TransferState{
        START,
        LATCH_OPEN_CLOSE_CLAW,
        SMOLANGLE,
        PIVOT_BACK_VIRTUAL_TO_TRANSFER,
        CLOSE_LATCH,
        MOPEN_CLAW,
        VIRTUAL_DOWN_PIVOT_FRONT,
        OPEN_CLAW,
        END,
        ABORT
    }

    public TransferState transferState;

    boolean rotated = false;

    void updateTransferState(){
        if(state!=State.TRANSFERING) transferState = TransferState.END;
        if(transferState == TransferState.END) return;
        switch (transferState){
            case START:
                rotated = false;
                if(claw.state!= Claw.State.CLOSED && claw.state!= Claw.State.MCLOSED)claw.setState(Claw.State.MCLOSING);
                transferState = TransferState.LATCH_OPEN_CLOSE_CLAW;
                latch.setState(Latch.State.OPENING);
                break;
            case LATCH_OPEN_CLOSE_CLAW:
                if(claw.state == Claw.State.MCLOSED || claw.state == Claw.State.CLOSED){
                    pivot.setState(VirtualPivot.State.RBACK);

                    uta.setState(UtaUta.State.SMOLAGNLINGFRONT);
                    transferState = TransferState.SMOLANGLE;
                    virtual.setState(Virtual.State.GOING_HOVER);
                }
                break;
            case SMOLANGLE:
                if(virtual.state == Virtual.State.HOVER && uta.state == UtaUta.State.SMOLANGLEFRONT){

                    virtual.setState(Virtual.State.GOING_TRANSFER);
                    transferState = TransferState.PIVOT_BACK_VIRTUAL_TO_TRANSFER;
                    uta.setState(UtaUta.State.SMOLANGELING);
                }
                break;
            case PIVOT_BACK_VIRTUAL_TO_TRANSFER:
                if(virtual.state == Virtual.State.TRANSFER && pivot.state == VirtualPivot.State.BACK && latch.state == Latch.State.OPENED && uta.state == UtaUta.State.SMOLANGLE){
                    latch.setState(Latch.State.CLOSING);
                    uta.setState(UtaUta.State.LEVELING);
                    transferState = TransferState.CLOSE_LATCH;
                }
                break;
            case CLOSE_LATCH:
                if(latch.state == Latch.State.CLOSED && uta.state == UtaUta.State.LEVEL){
                    claw.setState(Claw.State.MOPENING);
                    transferState = TransferState.MOPEN_CLAW;
                }
                break;
            case MOPEN_CLAW:
                if(claw.state == Claw.State.MOPENED){
                    rotated = false;
                    virtual.setState(Virtual.State.GOING_DOWN);
                    transferState = TransferState.VIRTUAL_DOWN_PIVOT_FRONT;
                }
                break;
            case VIRTUAL_DOWN_PIVOT_FRONT:
                if(virtual.virtualEncoder.getCurrentPosition() <= Virtual.rotatePositionFromBack && !rotated){
                    pivot.setState(VirtualPivot.State.RFRONT);
                    rotated = true;
                }
                if(virtual.state == Virtual.State.DOWN && pivot.state == VirtualPivot.State.FRONT){
                    claw.setState(Claw.State.OPENED);
                    transferState = TransferState.OPEN_CLAW;
                }
                break;
            case OPEN_CLAW:
                if(claw.state == Claw.State.OPENED){
                    transferState = TransferState.END;
                }
                break;
            case ABORT:
                if(claw.state != Claw.State.OPENED && claw.state != Claw.State.OPENING
                        && claw.state != Claw.State.MOPENED && claw.state != Claw.State.MOPENING) claw.setState(Claw.State.OPENING);
                if(virtual.state != Virtual.State.DOWN && virtual.state != Virtual.State.GOING_DOWN) virtual.setState(Virtual.State.GOING_DOWN);
                if(pivot.state != VirtualPivot.State.RFRONT && pivot.state != VirtualPivot.State.FRONT) pivot.setState(VirtualPivot.State.RFRONT);
                if(uta.state!= UtaUta.State.LEVEL && uta.state!= UtaUta.State.LEVELING) uta.setState(UtaUta.State.LEVELING);
                if(virtual.state == Virtual.State.DOWN && pivot.state == VirtualPivot.State.FRONT && (claw.state == Claw.State.OPENED || claw.state == Claw.State.MOPENED)&&uta.state == UtaUta.State.LEVEL) transferState = TransferState.END;
                break;
        }
        if(transferState == TransferState.END)
            Virtual.stackIndex = Math.max(Virtual.stackIndex - 1, 0);
    }

    @Override
    public void atStart() {
        pivot.atStart();
        uta.atStart();
        popa.atStart();
    }

    @Override
    public void loop() {
        updateState();
        updateTransferState();
        pivot.loop();
        uta.loop();
        popa.loop();
    }
}
