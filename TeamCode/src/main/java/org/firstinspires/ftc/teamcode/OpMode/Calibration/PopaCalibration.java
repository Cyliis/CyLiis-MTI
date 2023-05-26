package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group = "Calibration", name = "popa")
public class PopaCalibration extends LinearOpMode {

    Servo popa;

    double pos = 0.5;

    double pos1 = 0.5, pos2 = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);
        popa = hardwareMap.get(Servo.class, "funny");
        popa.setPosition(pos);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up) pos += 0.001;
            if(gamepad1.dpad_down) pos -= 0.001;
            telemetry.addData("Position", pos);
            popa.setPosition(pos);
            stickyGamepad.update();
            telemetry.update();
        }
    }
}
