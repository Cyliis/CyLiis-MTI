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

    DcMotorEx virtual;
    DcMotorEx encoder;

    public static double TICKS_PER_REV = 8192;
    public static int offset = -1100;

    int pos = 0;
    int tpos = 0;

    public static double p = 0.002,i = 0,d = 0.00005,f=0.125;

    FtcDashboard dash;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());

        encoder = hardwareMap.get(DcMotorEx.class, "encoder");
        virtual = hardwareMap.get(DcMotorEx.class, "virtual");
        virtual.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        virtual.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        virtual.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        virtual.setDirection(DcMotorSimple.Direction.REVERSE);
        encoder.setDirection(DcMotorSimple.Direction.REVERSE);
//        virtual.setPower(Virtual.virtualPower);
//        virtual.setTargetPosition(0);
//        virtual.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        PIDController pid = new PIDController(p,i,d);

        waitForStart();

        while(opModeIsActive()){
//            double power = 0;
            if(gamepad1.dpad_up)pos+=10;
            if(gamepad1.dpad_down)pos-=10;

            if(gamepad1.y) tpos = pos;

            pid.setPID(p,i,d);
//            pid.setTolerance(64);
            double power = pid.calculate(encoder.getCurrentPosition(), tpos);
            power += f*Math.cos((encoder.getCurrentPosition() + offset)/TICKS_PER_REV * 2*Math.PI);

            virtual.setPower((double)((int)(power*1000))/1000.0);
//            virtual.setTargetPosition(pos);

            telemetry.addData("pos", pos);
            telemetry.addData("current pos", encoder.getCurrentPosition());
            telemetry.addData("power", (double)((int)(power*1000))/1000.0);
            telemetry.addData("feedforward", f*Math.cos((encoder.getCurrentPosition() + offset)/TICKS_PER_REV * 2*Math.PI));
            telemetry.update();
        }
    }
}
