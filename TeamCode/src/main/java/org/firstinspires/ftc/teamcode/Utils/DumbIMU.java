package org.firstinspires.ftc.teamcode.Utils;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.OpMode.OpMode;

public class DumbIMU
{
    IMU imu;

    RevHubOrientationOnRobot orientationOnRobot;

    private final Object o = new Object();
    private double heading;
    private Thread threadIMU;

    public synchronized double getHeading(){
        synchronized (o) {
            return heading;
        }
    }

    public DumbIMU(HardwareMap hm, LinearOpMode op){
        imu = hm.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        this.orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);


    }

    private void newThread(LinearOpMode op){
        threadIMU = new Thread(()->{
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            while(op.opModeIsActive() && !op.isStopRequested()){
                synchronized (o){
                    YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                    heading = orientation.getYaw(AngleUnit.RADIANS);
                }
            }
        });
        threadIMU.start();
    }

    public void init(LinearOpMode op){
        if (threadIMU==null)newThread(op);
        else if(threadIMU.isAlive()){
            threadIMU.interrupt();
            newThread(op);
        }
    }
}