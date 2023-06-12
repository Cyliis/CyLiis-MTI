package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group = "Calibration", name = "odo")
public class OdoRetractionCalibration extends LinearOpMode {

    Servo odo;

    double pos = 0.5;

    double pos1 = 0, pos2 = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);
        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(pos);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up) pos += 0.001;
            if(gamepad1.dpad_down) pos -= 0.001;
            telemetry.addData("Position", pos);
            odo.setPosition(pos);
            stickyGamepad.update();
            telemetry.update();
        }
    }
}
