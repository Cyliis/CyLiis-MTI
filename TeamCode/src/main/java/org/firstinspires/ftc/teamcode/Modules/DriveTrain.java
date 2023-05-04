package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Utils.DumbIMU;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Vector;

import java.util.ArrayList;

@Config
public class DriveTrain implements IRobotModule {
    public static boolean ENABLE_MODULE = true;

    private final String MOTOR_FRONT_LEFT_NAME = "mfl";
    private final String MOTOR_FRONT_RIGHT_NAME = "mfr";
    private final String MOTOR_BACK_LEFT_NAME = "mbl";
    private final String MOTOR_BACK_RIGHT_NAME = "mbr";

    private DcMotorEx mfl;
    private DcMotorEx mfr;
    private DcMotorEx mbl;
    private DcMotorEx mbr;

    public enum DRIVE_MODE{
        ROBOT_CENTRIC, FIELD_CENTRIC
    }

    public DRIVE_MODE mode = DRIVE_MODE.FIELD_CENTRIC;

    public enum SPEED{
        SLOW(0.5),FAST(1);
        final double multiplier;

        SPEED(double multiplier) {
            this.multiplier = multiplier;
        }
    }

    public SPEED speed = SPEED.SLOW;

    public DumbIMU imu;
    public static double imuOffset;
    public double imuValue;

    public DriveTrain(HardwareMap hm){
        init(hm);
        initImu(hm);
    }

    private void init(HardwareMap hm){
        mfl = hm.get(DcMotorEx.class, MOTOR_FRONT_LEFT_NAME);
        mfr = hm.get(DcMotorEx.class, MOTOR_FRONT_RIGHT_NAME);
        mbl = hm.get(DcMotorEx.class, MOTOR_BACK_LEFT_NAME);
        mbr = hm.get(DcMotorEx.class, MOTOR_BACK_RIGHT_NAME);

//        mfl.setDirection(DcMotorSimple.Direction.REVERSE);
        mfr.setDirection(DcMotorSimple.Direction.REVERSE);
//        mbl.setDirection(DcMotorSimple.Direction.REVERSE);
        mbr.setDirection(DcMotorSimple.Direction.REVERSE);

        ArrayList<DcMotorEx> motorList = new ArrayList<>();
        motorList.add(mfl);motorList.add(mfr);
        motorList.add(mbl);motorList.add(mbr);

        for (DcMotorEx motor:motorList) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }
    }

    public static class DriveParameters {
        public double forward, right, clockwise;
        public DriveParameters(double forward, double right, double clockwise){
            this.forward = forward;
            this.right = right;
            this.clockwise = clockwise;
        }
        void normalize(){
            double denominator = Math.abs(forward)+Math.abs(right)+Math.abs(clockwise);
            denominator = Math.max(1, denominator);
            forward/=denominator;
            right/=denominator;
            clockwise/=denominator;
        }
        void setMultiplier(double multiplier){
            forward*=multiplier;
            right*=multiplier;
            clockwise*=multiplier;
        }
    }


    private void initImu(HardwareMap hm){
        imu = new DumbIMU(hm);
    }

    private void updateImu(){
        if(mode != DRIVE_MODE.FIELD_CENTRIC) return;
        imu.loop();
        imuValue = imu.getHeading();
    }

    private DriveParameters getParametersRC(DriveParameters initialDriveParameters){
        return initialDriveParameters;
    }

    private DriveParameters getParametersFC(DriveParameters initialDriveParameters){
        Vector translational = new Vector(initialDriveParameters.right, initialDriveParameters.forward);
        translational.set_angle_offset(-(imuValue - imuOffset));
        return new DriveParameters(translational.cy, translational.cx, initialDriveParameters.clockwise);
    }

    public void drive(double forward, double right, double clockwise){
        DriveParameters driveParameters = new DriveParameters(0,0,0);

        switch (mode){
            case FIELD_CENTRIC:
                driveParameters = getParametersFC(new DriveParameters(forward, right, clockwise));
                break;
            case ROBOT_CENTRIC:
                driveParameters = getParametersRC(new DriveParameters(forward, right, clockwise));
                break;
        }

        driveParameters.setMultiplier(speed.multiplier);

        driveForValues(driveParameters);
    }

    private void driveForValues(DriveParameters parameters){
        parameters.normalize();
        mfl.setPower(parameters.forward + parameters.right + parameters.clockwise);
        mfr.setPower(parameters.forward - parameters.right - parameters.clockwise);
        mbl.setPower(parameters.forward - parameters.right + parameters.clockwise);
        mbr.setPower(parameters.forward + parameters.right - parameters.clockwise);
    }

    @Override
    public void atStart() {

    }

    @Override
    public void loop() {
        updateImu();
    }
}
