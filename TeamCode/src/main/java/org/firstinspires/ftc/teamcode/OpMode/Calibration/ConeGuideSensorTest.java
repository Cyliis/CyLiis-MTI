package org.firstinspires.ftc.teamcode.OpMode.Calibration;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(group = "Calibration", name = "sensorTest")
public class ConeGuideSensorTest extends OpMode {

    DigitalChannel sensor;

    @Override
    public void init() {
        sensor = hardwareMap.get(DigitalChannel.class, "coneGuide");
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        telemetry.addData("Detected",sensor.getState());
        telemetry.update();
    }
}
