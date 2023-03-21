package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

@Config
public class Virtual implements IRobotModule {

    public static boolean ENABLE_MODULE = true;


    public static String VIRTUAL_NAME = "virtual";
    public static boolean reversed = false;
    public int ground;

    HardwareMap hm;
    NanoClock nanoClock;

    DcMotorEx virtual;

    public static int stack1 = 0, stack2 = 0, stack3 = 0, stack4 = 0, stack5 = 0;
    public static int[] stack = {stack1, stack2, stack3, stack4, stack5};
    public static int stackIndex = 0;
    public static int downPosition = stack[stackIndex], hoverPosition = 0, lowPosition = 0, transferPosition = 0;
    public double target = downPosition;

    public static double p,i,d;

    PIDController pid = new PIDController(p,i,d);

    public enum State{
        GOING_DOWN(downPosition),
        DOWN(downPosition),
        GOING_HOVER(hoverPosition),
        HOVER(hoverPosition),
        GOING_LOW(lowPosition),
        LOW(lowPosition),
        GOING_TRANSFER(transferPosition),
        TRANSFER(transferPosition);

        public int pos;
        State(int pos){this.pos = pos;}
    }

    public State state;

    public Virtual(HardwareMap hm, boolean resetEncoders){
        this.hm = hm;
        init(resetEncoders);
    }

    void init(boolean resetEncoders){
        virtual = hm.get(DcMotorEx.class, VIRTUAL_NAME);
        if(resetEncoders)resetEncoders();
        if(reversed) virtual.setDirection(DcMotorSimple.Direction.REVERSE);
        virtual.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        state = State.GOING_DOWN;
        nanoClock = NanoClock.system();
    }

    void resetEncoders(){
        virtual.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    double timeOfLastStateChange;

    void setState(State state){
        if(state == this.state) return;
        timeOfLastStateChange = nanoClock.seconds();
        this.state = state;
    }

    public static int virtualTolerance = 4;

    private void updateState(){
        switch (state){
            case GOING_DOWN:
                if(Math.abs(virtual.getCurrentPosition() - downPosition) <= virtualTolerance)
                    setState(State.DOWN);
                break;
            case GOING_HOVER:
                if(Math.abs(virtual.getCurrentPosition() - hoverPosition) <= virtualTolerance)
                    setState(State.HOVER);
                break;
            case GOING_LOW:
                if(Math.abs(virtual.getCurrentPosition() - lowPosition) <= virtualTolerance)
                    setState(State.LOW);
                break;
            case GOING_TRANSFER:
                if(Math.abs(virtual.getCurrentPosition() - transferPosition) <= virtualTolerance)
                    setState(State.TRANSFER);
                break;
        }
    }

    void updateMotors(){
        target = state.pos + ground;

        pid.setPID(p,i,d);

        double power = pid.calculate(virtual.getCurrentPosition(), target);

        virtual.setPower(power);
    }

    @Override
    public void atStart(){
        setState(State.GOING_DOWN);
    }

    @Override
    public void loop() {
        updateState();
        updateMotors();
    }
}
