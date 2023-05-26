package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@Config
@TeleOp(group = "Calibration", name = "lift")
public class LiftCalibration extends LinearOpMode {

    FtcDashboard dash;

    DcMotorEx lift1, lift2;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());

        lift1 = hardwareMap.get(DcMotorEx.class,
                "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setDirection(DcMotorEx.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            lift1.setPower(0);
            lift2.setPower(0);

            if(gamepad1.dpad_down){
                lift1.setPower(-1);
                lift2.setPower(-1);
            }
            if(gamepad1.dpad_up){
                lift1.setPower(1);
                lift2.setPower(1);
            }

            telemetry.addData("current pos 1", lift1.getCurrentPosition());
            telemetry.addData("current pos 2", lift2.getCurrentPosition());
            telemetry.update();
        }
    }
}
