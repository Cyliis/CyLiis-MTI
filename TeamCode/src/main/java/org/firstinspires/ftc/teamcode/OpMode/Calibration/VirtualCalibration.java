package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.Utils.DumbEncoder;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@TeleOp(group = "Calibration", name = "virtual")
public class VirtualCalibration extends LinearOpMode {

    public static String VIRTUAL_LEFT_NAME = "virtual1";
    public static String VIRTUAL_RIGHT_NAME = "virtual2";
    public static String VIRTUAL_ENCODER_NAME = "virtualEncoder";
    public static boolean reversed1 = true , reversed2 = false, reversedEnc = true;
    public static double TICKS_PER_REV = 8192;
    public static double ticksOffset = -1076;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.0004,0.046,0.000028,0.18);
    private PIDController pid = new PIDController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);

    Servo virtual1 , virtual2;
    public DumbEncoder virtualEncoder;

    double target = 0.5;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());

        virtual1 = hardwareMap.get(Servo.class, VIRTUAL_LEFT_NAME);
        if(reversed1) virtual1.setDirection(Servo.Direction.REVERSE);
        virtual2 = hardwareMap.get(Servo.class, VIRTUAL_RIGHT_NAME);
        if(reversed2) virtual2.setDirection(Servo.Direction.REVERSE);

        virtualEncoder = new DumbEncoder(hardwareMap, VIRTUAL_ENCODER_NAME);
        if(reversedEnc) virtualEncoder.setReversed(true);
        virtualEncoder.reset();

        waitForStart();

        boolean np = true;

        while(opModeIsActive()){

            if(gamepad1.dpad_up) target+=0.001;
            else if(gamepad1.dpad_down) target-=0.001;

            virtual1.setPosition(target);
            virtual2.setPosition(target);

            telemetry.addData("Target position", target);
            telemetry.addData("Current position", virtualEncoder.getCurrentPosition());

            telemetry.update();
        }
    }
}
