package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Utils.DumbEncoder;

@Config
@TeleOp(group = "Calibration", name = "virtualBuLu")
public class VirtualBuruL extends LinearOpMode {

    public static String VIRTUAL_LEFT_NAME = "virtual1";
    public static String VIRTUAL_RIGHT_NAME = "virtual2";
    public static String VIRTUAL_ENCODER_NAME = "virtual1";
    public static boolean reversed1 = false , reversed2 = true, reversedEnc = true;
    public static double TICKS_PER_REV = 8192;
    public static double ticksOffset = -1076;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0,0,0,0);
    private PIDController pid = new PIDController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);

    DcMotorEx virtual1 , virtual2;
    public DumbEncoder virtualEncoder;

    int target;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());

        virtual1 = hardwareMap.get(DcMotorEx.class, VIRTUAL_LEFT_NAME);
        if(reversed1) virtual1.setDirection(DcMotorEx.Direction.REVERSE);
        virtual1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        virtual2 = hardwareMap.get(DcMotorEx.class, VIRTUAL_RIGHT_NAME);
        if(reversed2) virtual2.setDirection(DcMotorEx.Direction.REVERSE);
        virtual2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        virtualEncoder = new DumbEncoder(hardwareMap, VIRTUAL_ENCODER_NAME);
        virtualEncoder.reset();
        if(reversedEnc) virtualEncoder.setReversed(true);

        waitForStart();

        boolean np = true;

        while(opModeIsActive()){

            if(gamepad1.dpad_down) target = 1000;
            else if(gamepad1.dpad_up) target = 2500;

            double ff, power;

            ff = Math.cos(((double)virtualEncoder.getCurrentPosition() + ticksOffset) / TICKS_PER_REV * (Math.PI * 2)) * pidfCoefficients.f;
            pid.setPID(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);
            power = pid.calculate(virtualEncoder.getCurrentPosition(), target);
            virtual1.setPower(ff+power);
            virtual2.setPower(ff+power);

            telemetry.addData("Feed forward", ff);
            telemetry.addData("Power", power);
            telemetry.addData("Target position", target);
            telemetry.addData("Current position", virtualEncoder.getCurrentPosition());

            telemetry.update();
        }
    }
}
