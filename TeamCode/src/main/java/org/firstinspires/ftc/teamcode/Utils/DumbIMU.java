package org.firstinspires.ftc.teamcode.Utils;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DumbIMU implements IRobotModule
{
    IMU imu;

    RevHubOrientationOnRobot orientationOnRobot;

    public double heading;


    public DumbIMU(HardwareMap hm){
        imu = hm.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        this.orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
    }

    public void init(){
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @Override
    public void atStart() {

    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        heading = orientation.getYaw(AngleUnit.RADIANS);
    }
}