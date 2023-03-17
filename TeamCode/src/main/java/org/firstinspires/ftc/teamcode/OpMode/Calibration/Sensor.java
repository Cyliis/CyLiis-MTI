package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.android.dx.dex.file.StringIdItem;
import org.firstinspires.ftc.teamcode.Modules.DistanceSensor;

@Config
@TeleOp(group = "Calibration", name = "sensorTest")
public class Sensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor sensor = new DistanceSensor(hardwareMap);
        NanoClock nanoClock = NanoClock.system();
        double timestamp = 0;
        waitForStart();
        while(opModeIsActive()){
            sensor.loop();
            telemetry.addData("distance", sensor.value);
            telemetry.addData("time", (int)((nanoClock.seconds() - timestamp) * 1000));
            timestamp = nanoClock.seconds();
            telemetry.update();
        }
    }
}
