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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Utils.DumbEncoder;

@Config
@TeleOp(group = "Calibration", name = "Lift bruh")
public class SlowLiftCalibration extends LinearOpMode {

    FtcDashboard dash;

    public static String LIFT1_NAME = "lift1";
    public static String LIFT2_NAME = "lift2";
    public static boolean reversed1 = true, reversed2 =false, reversedEnc = true;

    NanoClock nanoClock;

    public DcMotorEx lift1, lift2;
    public DumbEncoder liftEncoder;

    public static PIDCoefficients pidCoefficients =  new PIDCoefficients(0.01,0.12, 0.0008);
    public static double f1 = 0.12, f2 = 0.06;
    public static double maxPos = 630;
    PIDController pid = new PIDController(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);

    public static int target = 0;

    private void initialise(){
        lift1 = hardwareMap.get(DcMotorEx.class, LIFT1_NAME);
        lift2 = hardwareMap.get(DcMotorEx.class, LIFT2_NAME);

        if(reversed1) lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorConfigurationType motorConfigurationType = lift1.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lift1.setMotorType(motorConfigurationType);

        if(reversed2) lift2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorConfigurationType = lift2.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        lift2.setMotorType(motorConfigurationType);

        liftEncoder = new DumbEncoder(hardwareMap,"mfr", reversedEnc);
        liftEncoder.setReversed(reversedEnc);
        liftEncoder.reset();

        nanoClock = NanoClock.system();
    }

    double timeStamp;

    void funny(){
        timeStamp = timeStamp - 1 + 1;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialise();

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());

        waitForStart();

        while(opModeIsActive()){

            timeStamp = nanoClock.seconds();

            int current = liftEncoder.getCurrentPosition();

            pid.setPID(pidCoefficients.p, pidCoefficients.i, pidCoefficients.d);

            double power = pid.calculate(liftEncoder.getCurrentPosition(), target) + f1*((double)current/(double)maxPos) + f2;

            lift1.setPower(power);
            lift2.setPower(power);

            telemetry.addData("current pos", liftEncoder.getCurrentPosition());
            telemetry.addData("target pos", target);
            telemetry.update();

            while(nanoClock.seconds() < timeStamp+0.02) funny();

        }
    }
}
