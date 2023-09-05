package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.AsymmetricMotionProfile;

@TeleOp(name = "Profile Test")
@Config
public class ProfileTest extends OpMode {

    AsymmetricMotionProfile profile = new AsymmetricMotionProfile(0,0,0);

    public static double velocity = 20, acceleration = 10, deceleration = 4;

    public double currentPos, currentVelocity;
    public static double targetPos = 300;

    FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry,dashboard.getTelemetry());
    }

    boolean pressed = false;

    int updates;
    @Override
    public void loop() {
        if(gamepad1.x && !pressed){
            profile = new AsymmetricMotionProfile(velocity, acceleration, deceleration);
            profile.setMotion(currentPos, targetPos, currentVelocity);
            updates++;
        }
        pressed = gamepad1.x;

        profile.update();

        currentPos = profile.getPosition();
        currentVelocity = 0;

        profile.telemetry(telemetry);

        telemetry.addData("Current Position", currentPos);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("Target Position", targetPos);
        telemetry.addData("Updates", updates);
    }
}
