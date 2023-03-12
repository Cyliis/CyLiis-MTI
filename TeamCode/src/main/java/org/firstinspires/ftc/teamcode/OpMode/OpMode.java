package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;
import java.util.List;


@TeleOp(name="OpMode👉👌")
public class OpMode extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;
    DriveTrain driveTrain;
    RobotModules robotModules;
    TiedBehaviour tiedBehaviour;

    public void initialize(){
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        driveTrain = new DriveTrain(hardwareMap, gamepad1, DriveTrain.DriveMode.HEADLESS);
        robotModules = new RobotModules(hardwareMap);
        tiedBehaviour = new TiedBehaviour(gamepad1, gamepad2, robotModules, driveTrain);

        nanoClock = NanoClock.system();
    }

    @Override
    public void runOpMode()  {
        initialize();
        waitForStart();

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        robotModules.atStart();

        while(opModeIsActive()) {
            double timeMs = nanoClock.seconds()*1000;

            for(LynxModule hub:hubs)
                hub.clearBulkCache();
            driveTrain.loop();
            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loop time:" , (int)(nanoClock.seconds()*1000 - timeMs));

            telemetry.update();
        }

        robotModules.emergencyStop();
    }
}