package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Latch;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group = "Calibration", name = "latch")
public class LatchCalibration extends LinearOpMode {

    Servo latch;

    double pos = 0.5;
    int posIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);
        latch = hardwareMap.get(Servo.class, "latch");
        latch.setPosition(pos);
        waitForStart();
        while(opModeIsActive()){
            if(posIndex == 0){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Open Position");
                if(stickyGamepad.y){
                    Latch.openedlatchPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex == 1){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Closed Position");
                if(stickyGamepad.y){
                    Latch.closedlatchPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex>1) break;
            latch.setPosition(pos);
            stickyGamepad.update();
            telemetry.update();
        }
    }
}
