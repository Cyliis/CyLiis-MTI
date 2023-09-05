package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.CoolVector;

@TeleOp(name = "Test mecanum")
public class TestMecanum extends OpMode {

    MecanumDrive mecanumDrive;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, null, MecanumDrive.RunMode.Vector);
    }

    @Override
    public void loop() {
        mecanumDrive.setTargetVector(new CoolVector(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.left_trigger - gamepad1.right_trigger));
        mecanumDrive.loop();
        telemetry.addData("output x", mecanumDrive.powerCoolVector.getX());
        telemetry.addData("output y", mecanumDrive.powerCoolVector.getY());
        telemetry.addData("output h", mecanumDrive.powerCoolVector.getZ());
        telemetry.addData("input x", mecanumDrive.targetCoolVector.getX());
        telemetry.addData("input y ", mecanumDrive.targetCoolVector.getY());
        telemetry.addData("input h", mecanumDrive.targetCoolVector.getZ());
        telemetry.update();
    }
}
