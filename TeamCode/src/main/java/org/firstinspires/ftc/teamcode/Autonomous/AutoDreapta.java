package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.DriveTrainControlTriggers;
import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.Standard;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;


@Autonomous(name="Auto dreapta👉👌")
public class AutoDreapta extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    SampleMecanumDrive driveTrain;
    RobotModules robotModules;
    Standard gamepadControl;
    TiedBehaviour tiedBehaviour;

    Servo odo;

    public void initialize(){
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs) {
            if(hub.getImuType() == LynxModuleImuType.BHI260) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            else hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        robotModules = new RobotModules(hardwareMap, true);
        gamepadControl = new Standard(gamepad1, gamepad2, robotModules);
        tiedBehaviour = new TiedBehaviour(robotModules);

        driveTrain = new SampleMecanumDrive(hardwareMap);

        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(0.8);

        nanoClock = NanoClock.system();

        PhotonCore.enable();
    }

    @Override
    public void runOpMode()  {
        initialize();

        waitForStart();

        driveTrain.imu.startIMUThread(this);

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        robotModules.atStart();

        while(opModeIsActive() && !isStopRequested()) {
            double timeMs = nanoClock.seconds()*1000;

            for(LynxModule hub:hubs)
                hub.clearBulkCache();

            driveTrain.update();

            gamepadControl.loop();
            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , (int)(1000)/(nanoClock.seconds()*1000 - timeMs));
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        robotModules.emergencyStop();
    }
}