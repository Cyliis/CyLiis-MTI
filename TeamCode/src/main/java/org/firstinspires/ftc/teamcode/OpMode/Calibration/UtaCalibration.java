package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.UtaUta;
import org.firstinspires.ftc.teamcode.Modules.VirtualPivot;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;

@TeleOp(group = "Calibration", name = "uta")
public class UtaCalibration extends LinearOpMode {

    Servo uta;

    double pos = 0.5;
    int posIndex = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);
        uta = hardwareMap.get(Servo.class, "uta");
        uta.setPosition(pos);
        waitForStart();
        while(opModeIsActive()){
            if(posIndex == 0){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Leveled Position");
                if(stickyGamepad.y){
                    UtaUta.levelPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex == 1){
                if(gamepad1.dpad_up) pos+= 0.0001;
                if(gamepad1.dpad_down) pos-= 0.0001;
                telemetry.addData("Position", pos);
                telemetry.addLine("Press Y to set Angled Position");
                if(stickyGamepad.y){
                    UtaUta.angledPosition = pos;
                    posIndex++;
                }
            }
            else if(posIndex>1) break;
            uta.setPosition(pos);
            stickyGamepad.update();
            telemetry.update();
        }
    }
}
