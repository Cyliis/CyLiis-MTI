package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.DriveTrainControlTriggers;
import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.Standard;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;

import java.util.List;


@TeleOp(name="DP👉👌")
public class DriverPractice extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    DriveTrain driveTrain;
    DriveTrainControlTriggers dtControl;
    RobotModules robotModules;
    Standard gamepadControl;
    TiedBehaviour tiedBehaviour;

    Servo odo;

    public void initialize(){
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robotModules = new RobotModules(hardwareMap, true);
        gamepadControl = new Standard(gamepad1, gamepad2, robotModules);
        tiedBehaviour = new TiedBehaviour(robotModules, driveTrain);

        driveTrain = new DriveTrain(hardwareMap);
        dtControl = new DriveTrainControlTriggers(gamepad1, gamepad2, driveTrain, robotModules);

        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(0.8);

        nanoClock = NanoClock.system();

//        PhotonCore.experimental.setMaximumParallelCommands(8);
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

            driveTrain.loop();
            dtControl.loop();

            gamepadControl.loop();
            if(DriveTrain.ENABLE_MODULE)driveTrain.loop();
            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , (int)(1000)/(nanoClock.seconds()*1000 - timeMs));
            telemetry.addData("Drive mode", driveTrain.mode);
            telemetry.addData("Drive speed", driveTrain.speed);
            telemetry.addData("Imu value", driveTrain.imuValue);
            telemetry.addData("Bruh", PhotonCore.EXPANSION_HUB.getBulkCachingMode());
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        robotModules.emergencyStop();
    }
}