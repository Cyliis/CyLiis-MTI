package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@TeleOp(group = "Calibration", name = "virtual")
public class VirtualCalibration extends LinearOpMode {

    Servo virtual1, virtual2;
    DcMotorEx encoder;

    double pos = 0.5;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());

        encoder = hardwareMap.get(DcMotorEx.class, Virtual.VIRTUAL_ENCODER_NAME);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(Virtual.reversedEnc) encoder.setDirection(DcMotorSimple.Direction.REVERSE);

        virtual1 = hardwareMap.get(Servo.class, Virtual.VIRTUAL_LEFT_NAME);
        virtual2 = hardwareMap.get(Servo.class, Virtual.VIRTUAL_RIGHT_NAME);
        if(Virtual.reversed1) virtual1.setDirection(Servo.Direction.REVERSE);
        if(Virtual.reversed2) virtual2.setDirection(Servo.Direction.REVERSE);

        virtual1.setPosition(pos);
        virtual2.setPosition(pos);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_up)pos+=0.0001;
            if(gamepad1.dpad_down)pos-=0.0001;

            virtual1.setPosition(pos);
            virtual2.setPosition(pos);

            telemetry.addData("pos", pos);
            telemetry.addData("current encoder pos", encoder.getCurrentPosition());


            telemetry.addLine("PRIMA LINIE E POZITIA SERVOULUI SI A DOUA A ENCODERULUI, INLOCUIESTE IN COD PT CE POZITIE VREI SA FIE");
            telemetry.addLine("PUNE Virtual.rotatePositionFromFrontE SI Virtual.rotatePositionFromBackE PE LA MIJLOC INTRE DOWN SI TRANSFER SI CALIBREZ EU MAI BINE MAINE DACA NU E NEVOIE");
            telemetry.addLine("AI VARIABILE BOOLEAN DE REVERSE DACA E NEVOIE FOLOSESTELE");

            telemetry.update();
        }
    }
}
