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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Utils.DumbEncoder;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

@Config
@TeleOp(group = "Calibration", name = "virtualBuLu")
public class VirtualBuruL extends LinearOpMode {

    public static String VIRTUAL_LEFT_NAME = "virtual1";
    public static String VIRTUAL_RIGHT_NAME = "virtual2";
    public static String VIRTUAL_ENCODER_NAME = "virtual1";
    public static boolean reversed1 = true , reversed2 = false, reversedEnc = false;
    public static double TICKS_PER_REV = 8192;
    public static double ticksOffset = -1076;

    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.00125,0.065,0.00006,0.14);
    private PIDController pid = new PIDController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);

    DcMotorEx virtual1 , virtual2;
    public DumbEncoder virtualEncoder;

    int index = 0;
    ArrayList<Integer> poses = new ArrayList<>();
    int target = 0;
    int fun = 0;

    FtcDashboard dash;

    NanoClock nanoClock;

    boolean np1 = true;
    boolean np2 = true;
    boolean np3 = true;

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

        nanoClock = NanoClock.system();

        waitForStart();

        poses.add(0);

        double stamp = nanoClock.seconds();

        while(opModeIsActive()){

            if(gamepad1.dpad_up) fun++;
            else if(gamepad1.dpad_down) fun--;

            if(gamepad1.x && np1) {
                np1 = false;
                poses.add(fun);
                Collections.sort(poses);
            } else if(!gamepad1.x) np1 = true;

            if(gamepad1.left_bumper && np2) {
                index--;
                np2 = false;
            } else if(!gamepad1.left_bumper) np2 = true;
            if(gamepad1.right_bumper && np3) {
                index++;
                np3 = false;
            } else if(!gamepad1.right_bumper) np3 = true;

            index = Math.max(0, index);
            index = Math.min(poses.size() - 1, index);

            target = poses.get(index);

            double ff, power;

            ff = Math.cos(((double)virtualEncoder.getCurrentPosition() + ticksOffset) / TICKS_PER_REV * (Math.PI * 2)) * pidfCoefficients.f;
            pid.setPID(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d);
            power = pid.calculate(virtualEncoder.getCurrentPosition(), target);
            virtual1.setPower(ff+power);
            virtual2.setPower(ff+power);

            telemetry.addData("Loops per sec", 1.0/(nanoClock.seconds() - stamp));
            telemetry.addData("Fun", fun);
            telemetry.addData("Feed forward", ff);
            telemetry.addData("Power", power);
            telemetry.addData("Target position", target);
            telemetry.addData("Current position", virtualEncoder.getCurrentPosition());
            telemetry.addData("Poses", poses.size());
            telemetry.addData("Index", index);

            telemetry.update();

            stamp = nanoClock.seconds();
        }
    }
}
