package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group = "Calibration", name = "virtual")
public class VirtualCalibration extends LinearOpMode {

    Servo virtual1, virtual2;

    double pos = 0.5;
    int posIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);
        virtual1 = hardwareMap.get(Servo.class, "virtual1");
        virtual2 = hardwareMap.get(Servo.class, "virtual2");
        virtual2.setDirection(Servo.Direction.REVERSE);
        virtual1.setPosition(pos);
        virtual2.setPosition(pos);

        waitForStart();

        while(opModeIsActive()){
            if(posIndex == 0){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Down Position");
                if(stickyGamepad.y){
                    Virtual.downPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex == 1){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Hover Position");
                if(stickyGamepad.y){
                    Virtual.hoverPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex == 2){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Rotate Position");
                if(stickyGamepad.y){
                    Virtual.rotatePosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex == 3){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Transfer Position");
                if(stickyGamepad.y){
                    Virtual.transferPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex>3) break;
            virtual1.setPosition(pos);
            virtual2.setPosition(pos);
            stickyGamepad.update();
            telemetry.update();
        }
    }
}
