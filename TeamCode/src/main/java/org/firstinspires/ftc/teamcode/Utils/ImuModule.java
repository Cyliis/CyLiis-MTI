package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ImuModule {

    private final Object imuLock1 = new Object();
    private final Object imuLock2 = new Object();
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
            imu.resetYaw();
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                synchronized (imuLock1) {
                    imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                }
                synchronized (imuLock2){
                    imuVelocity = (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
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