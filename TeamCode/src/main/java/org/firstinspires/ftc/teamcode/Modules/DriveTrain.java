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

    public static double LAST_ANGLE_READ;

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
    double angleAtReset = 0;

    public DumbIMU imu;

    final private HardwareMap hm;

    final private Gamepad gamepad;

    private final LinearOpMode op;

    public DriveTrain(HardwareMap hm, Gamepad gamepad, DriveMode mode, LinearOpMode op){
        this.hm = hm;
        this.gamepad = gamepad;
        DRIVE_MODE = mode;
        this.op = op;
        init();
        imu = new DumbIMU(hm, op);
    }

    private void init(){
        frontLeft = hm.get(DcMotor.class, MOTOR_FRONT_LEFT_NAME);
        frontRight = hm.get(DcMotor.class, MOTOR_FRONT_RIGHT_NAME);
        backLeft = hm.get(DcMotor.class, MOTOR_BACK_LEFT_NAME);
        backRight = hm.get(DcMotor.class, MOTOR_BACK_RIGHT_NAME);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void reset_imu(LinearOpMode op){
        imu.init(op);
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

    boolean np2 = true;

    void reset(){
        if(gamepad.back && np2) {
            np2 = false;
            LAST_ANGLE_READ = 0;
            reset_imu(op);
        }
        if(!gamepad.back) np2 = false;
    }

    private void driveNormaly(){
        this.forward = -gamepad.left_stick_y;
        this.right = gamepad.left_stick_x;
        this.rotateClockwise = gamepad.right_trigger - gamepad.left_trigger;
    }

    private void driveHeadlessly(){
        movementVector.set_components(gamepad.left_stick_x, -gamepad.left_stick_y);

        this.rotateClockwise = gamepad.right_trigger - gamepad.left_trigger;

        double angle = imu.getHeading();

        LAST_ANGLE_READ = angle;
        angle += angleAtReset;

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
        change();
        boost();
        reset();

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
