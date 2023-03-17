package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.VirtualPivot;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group = "Calibration", name = "pivot")
public class PivotCalibration extends LinearOpMode {

    Servo pivot;

    double pos = 0.5;
    int posIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);
        pivot = hardwareMap.get(Servo.class, "pivot");
        pivot.setPosition(pos);
        waitForStart();
        while(opModeIsActive()){
            if(posIndex == 0){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Front Position");
                if(stickyGamepad.y){
                    VirtualPivot.frontPivotPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex == 1){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Back Position");
                if(stickyGamepad.y){
                    VirtualPivot.backPivotPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex>1) break;
            pivot.setPosition(pos);
            stickyGamepad.update();
            telemetry.update();
        }
    }
}
