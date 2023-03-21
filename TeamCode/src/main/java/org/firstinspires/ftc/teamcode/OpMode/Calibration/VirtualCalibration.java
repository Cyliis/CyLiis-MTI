package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group = "Calibration", name = "virtual")
public class VirtualCalibration extends LinearOpMode {

    DcMotorEx virtual;

    int pos = 0;

    public static double p = 0,i = 0,d = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        virtual = hardwareMap.get(DcMotorEx.class, "virtual");
        virtual.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        virtual.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDController pid = new PIDController(p,i,d);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up)pos++;
            if(gamepad1.dpad_down)pos--;

            pid.setPID(p,i,d);
            double power = pid.calculate(virtual.getCurrentPosition(), pos);

            virtual.setPower(power);
            virtual.setPower(power);

            telemetry.addData("pos", pos);
            telemetry.addData("current pos", virtual.getCurrentPosition());
            telemetry.update();
        }
    }
}
