package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuModule {

    private final Object imuLock = new Object();
    private final IMU imu;
    public static double imuAngle = 0;
    public static double imuVelocity = 0;

    public ImuModule(HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));
        imu.initialize(parameters);

        imu.resetYaw();
    }

    public void startIMUThread(LinearOpMode opMode) {
        new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (imuLock) {
                    imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    imuVelocity = (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate;
                }
            }
        }).start();
    }

    public double getHeading() {
        return imuAngle;
    }
    public double getVelocity() {
        return imuVelocity;
    }

}