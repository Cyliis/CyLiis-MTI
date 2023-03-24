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
    public int ground = 0;
    public static double TICKS_PER_REV = 8192;
    public static int offset = -3500;


    HardwareMap hm;
    NanoClock nanoClock;

    DcMotorEx virtual;
    DcMotorEx virtualEncoder;

    public static int stack1 = 300, stack2 = 90, stack3 = 150, stack4 = 200, stack5 = 320;
    public static int[] stack = {stack1, stack2, stack3, stack4, stack5};
    public static int stackIndex = 0;
    public static int downPosition = stack[stackIndex], hoverPosition = 700, lowPosition = 2000, rotatePositionFront = 800, rotatePositionBack = 3500, transferPosition = 4050;
    public int target = downPosition;
    public static double virtualPower = 1;

    public static double p = 0.002,i = 0,d = 0.00005,f=0.125;

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
        virtualEncoder = hm.get(DcMotorEx.class, "encoder");
        if(resetEncoders)resetEncoders();
        if(reversed) virtual.setDirection(DcMotorSimple.Direction.REVERSE);
        virtualEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
//        virtual.setPower(virtualPower);
//        virtual.setTargetPosition(downPosition);
//        virtual.setTargetPositionTolerance(target_tolerance);
//        virtual.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        virtual.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        state = State.GOING_DOWN;
        nanoClock = NanoClock.system();
    }

    void resetEncoders(){
        virtualEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    double timeOfLastStateChange;

    void setState(State state){
        if(state == this.state) return;
        timeOfLastStateChange = nanoClock.seconds();
        this.state = state;
    }

    public static int virtualTolerance = 200;

    private void updateState(){
        switch (state){
            case GOING_DOWN:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.pos) <= virtualTolerance)
                    setState(State.DOWN);
                break;
            case GOING_HOVER:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.pos) <= virtualTolerance)
                    setState(State.HOVER);
                break;
            case GOING_LOW:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.pos) <= virtualTolerance)
                    setState(State.LOW);
                break;
            case GOING_TRANSFER:
                if(Math.abs(virtualEncoder.getCurrentPosition() - state.pos) <= virtualTolerance)
                    setState(State.TRANSFER);
                break;
        }
    }

    void updateMotors(){
        target = state.pos + ground;

        pid.setPID(p,i,d);

        double power = pid.calculate(virtualEncoder.getCurrentPosition(), target);
        power += f*Math.cos((virtualEncoder.getCurrentPosition() + offset)/TICKS_PER_REV * 2*Math.PI);

        virtual.setPower(power);

//        virtual.setTargetPosition(target);
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
