package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GamepadControl;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;

import java.util.List;

import javax.xml.parsers.FactoryConfigurationError;


@TeleOp(name="DP👉👌")
public class DriverPractice extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    DriveTrain driveTrain;
    RobotModules robotModules;
    GamepadControl gamepadControl;
    TiedBehaviour tiedBehaviour;

    Servo odo;

    public void initialize(){

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        if(DriveTrain.ENABLE_MODULE)driveTrain = new DriveTrain(hardwareMap, gamepad1, DriveTrain.DriveMode.HEADLESS);
        robotModules = new RobotModules(hardwareMap, true);
        gamepadControl = new GamepadControl(gamepad1, gamepad2, robotModules);
        tiedBehaviour = new TiedBehaviour(robotModules, driveTrain);

        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(0.8);

        nanoClock = NanoClock.system();
    }

    @Override
    public void runOpMode()  {
        initialize();
        waitForStart();

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        robotModules.atStart();

        while(opModeIsActive() && !isStopRequested()) {
            double timeMs = nanoClock.seconds()*1000;

            for(LynxModule hub:hubs)
                hub.clearBulkCache();

            gamepadControl.loop();
            if(DriveTrain.ENABLE_MODULE)driveTrain.loop();
            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , (int)(1000)/(nanoClock.seconds()*1000 - timeMs));
            telemetry.addData("Imu value", driveTrain.imuValue);
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        robotModules.emergencyStop();
    }
}