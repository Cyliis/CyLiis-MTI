package org.firstinspires.ftc.teamcode.OpMode.Calibration;

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

    @Override
    public void runOpMode() throws InterruptedException {
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setDirection(DcMotorEx.Direction.REVERSE);
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setTargetPosition(pos);
        lift2.setTargetPosition(pos);
        lift1.setPower(1);
        lift2.setPower(1);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up)pos++;
            if(gamepad1.dpad_down)pos--;
            lift1.setTargetPosition(pos);
            lift2.setTargetPosition(pos);
            stickyGamepad.update();
            telemetry.addData("pos", pos);
            telemetry.update();
        }
    }
}
