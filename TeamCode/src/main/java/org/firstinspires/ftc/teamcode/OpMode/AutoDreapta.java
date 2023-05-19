package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.DriveTrainControlTriggers;
import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.Standard;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectories.AutoDreaptaTrajectories;

import java.util.List;


@Autonomous(name="Auto dreapta👉👌")
public class AutoDreapta extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    SampleMecanumDrive driveTrain;
    AutoDreaptaTrajectories trajectories;
    RobotModules robotModules;
    TiedBehaviour tiedBehaviour;

    Servo odo;

    public void initialize(){

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        robotModules = new RobotModules(hardwareMap, true);
        tiedBehaviour = new TiedBehaviour(robotModules);

        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(0);

        driveTrain = new SampleMecanumDrive(hardwareMap);
        trajectories = new AutoDreaptaTrajectories(driveTrain, robotModules);

        driveTrain.followTrajectorySequenceAsync(trajectories.preloadTrajectory());

        nanoClock = NanoClock.system();
    }

    int index = 0;

    @Override
    public void runOpMode()  {
        initialize();
        waitForStart();

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        robotModules.atStart();

        boolean picked = false;

        while(opModeIsActive() && !isStopRequested()) {
            double timeMs = nanoClock.seconds()*1000;

            for(LynxModule hub:hubs)
                hub.clearBulkCache();
            if(!driveTrain.isBusy()){
                if(index < 5){
                    if(!picked){
                        if(robotModules.outtake.state == Outtake.State.HIGH) {
                            robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                            driveTrain.followTrajectorySequenceAsync(trajectories.pickupTrajectory(index));
                            picked = true;
                        }
                    }
                    else {
                        driveTrain.followTrajectorySequenceAsync(trajectories.releaseTrajectory(index));
                        index++;
                        picked = false;
                    }
                }
            }

            driveTrain.update();

            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , (int)(1000)/(nanoClock.seconds()*1000 - timeMs));
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        robotModules.emergencyStop();
    }
}