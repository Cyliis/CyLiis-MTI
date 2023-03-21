package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group = "Calibration", name = "lift")
public class LiftCalibration extends LinearOpMode {

    DcMotorEx lift1, lift2;

    int pos = 0;

    public static double p = 0,i = 0,d = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setDirection(DcMotorEx.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDController pid = new PIDController(p,i,d);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up)pos++;
            if(gamepad1.dpad_down)pos--;

            pid.setPID(p,i,d);
            double power = pid.calculate(lift1.getCurrentPosition(), pos);

            lift1.setPower(power);
            lift2.setPower(power);

            telemetry.addData("pos", pos);
            telemetry.addData("current pos", lift1.getCurrentPosition());
            telemetry.update();
        }
    }
}
