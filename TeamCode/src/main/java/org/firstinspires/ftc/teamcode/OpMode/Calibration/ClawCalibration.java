package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group = "Calibration", name = "claw")
public class ClawCalibration extends LinearOpMode {

    Servo claw;

    double pos = 0.5;
    int posIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);
        claw = hardwareMap.get(Servo.class, "claw");
        claw.setPosition(pos);
        waitForStart();
        while(opModeIsActive()){
            if(posIndex == 0){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Open Position");
                if(stickyGamepad.y){
                    Claw.openedClawPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex == 1){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Closed Position");
                if(stickyGamepad.y){
                    Claw.closedClawPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex>1) break;
            claw.setPosition(pos);
            stickyGamepad.update();
            telemetry.update();
        }
    }
}
