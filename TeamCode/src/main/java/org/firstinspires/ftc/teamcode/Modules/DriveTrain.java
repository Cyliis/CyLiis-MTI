package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.DumbIMU;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
public class DriveTrain implements IRobotModule {

    public static boolean ENABLE_MODULE = true;

    public double multiplier = 0.5;

    private final String MOTOR_FRONT_LEFT_NAME = "mfl";
    private final String MOTOR_FRONT_RIGHT_NAME = "mfr";
    private final String MOTOR_BACK_LEFT_NAME = "mbl";
    private final String MOTOR_BACK_RIGHT_NAME = "mbr";

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    public enum DriveMode {
        NORMAL, HEADLESS;
    }

    public static DriveMode DRIVE_MODE;

    Vector movementVector = new Vector(0,0);

    double forward= 0, right = 0, rotateClockwise = 0;

    private DumbIMU imu = null;

    final private HardwareMap hm;

    final private Gamepad gamepad;

    public double imuValue = 0;

    private double headingDelta = 0;

    public DriveTrain(HardwareMap hm, Gamepad gamepad, DriveMode mode){
        this.hm = hm;
        this.gamepad = gamepad;
        DRIVE_MODE = mode;
        init();
//        thread.start();
    }

    private void init(){
        frontLeft = hm.get(DcMotor.class, MOTOR_FRONT_LEFT_NAME);
        frontRight = hm.get(DcMotor.class, MOTOR_FRONT_RIGHT_NAME);
        backLeft = hm.get(DcMotor.class, MOTOR_BACK_LEFT_NAME);
        backRight = hm.get(DcMotor.class, MOTOR_BACK_RIGHT_NAME);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = new DumbIMU(hm);
        imu.loop();
        headingDelta = imu.heading;
    }

    void boost(){
        if(gamepad.b)multiplier = 1;
        else multiplier = 0.5;
    }

    boolean changeNotPressed = true;

    void change(){
        if(gamepad.y && changeNotPressed){
            changeNotPressed = false;
            if(DRIVE_MODE == DriveMode.HEADLESS) DRIVE_MODE = DriveMode.NORMAL;
            else DRIVE_MODE = DriveMode.HEADLESS;
        }
        else if(!gamepad.y) changeNotPressed = true;
    }

    void reset(){
        if(gamepad.left_bumper && gamepad.right_bumper) {
            headingDelta = imu.heading;
        }
    }

    private void driveNormaly(){
        this.forward = -gamepad.left_stick_y;
        this.right = gamepad.left_stick_x;
        this.rotateClockwise = gamepad.right_trigger - gamepad.left_trigger;
    }

    private void driveHeadlessly(){
        movementVector.set_components(gamepad.left_stick_x, -gamepad.left_stick_y);

        this.rotateClockwise = gamepad.right_trigger - gamepad.left_trigger;

        double angle = imuValue;

        movementVector.set_angle_offset(Math.PI*2.0-angle);

        this.right = movementVector.cx;
        this.forward = movementVector.cy;
    }

    void driveForValues(double forward, double right, double rotateClockwise){
        forward = forward*forward* Math.signum(forward);
        right = right*right*Math.signum(right);
        rotateClockwise = rotateClockwise*rotateClockwise*Math.signum(rotateClockwise);

        double denomiantor = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(rotateClockwise),1);

        frontLeft.setPower(((forward + right + rotateClockwise) / denomiantor) * multiplier);
        frontRight.setPower(((forward - right - rotateClockwise) / denomiantor) * multiplier);
        backLeft.setPower(((forward - right + rotateClockwise) / denomiantor) * multiplier);
        backRight.setPower(((forward + right - rotateClockwise) / denomiantor) * multiplier);
    }

    @Override
    public void atStart() {
    }

    @Override
    public void loop() {
        imu.loop();
        change();
        boost();
        reset();

        imuValue = imu.heading - headingDelta;

        if(DRIVE_MODE == DriveMode.HEADLESS) driveHeadlessly();
        else driveNormaly();

        driveForValues(forward,right,rotateClockwise);
    }

    @Override
    public void emergencyStop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
