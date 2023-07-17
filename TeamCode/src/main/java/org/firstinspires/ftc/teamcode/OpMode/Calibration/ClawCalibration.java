package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@Config
@TeleOp(group = "Calibration", name = "claw")
public class ClawCalibration extends LinearOpMode {

    Servo claw;

    public static double pos = 0.5;
    int posIndex = 0;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(pos);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up) pos+= 0.0001;
            if(gamepad1.dpad_down) pos-= 0.0001;
            telemetry.addData("Position", pos);
            claw.setPosition(pos);
            stickyGamepad.update();
            telemetry.update();
        }
    }
}
