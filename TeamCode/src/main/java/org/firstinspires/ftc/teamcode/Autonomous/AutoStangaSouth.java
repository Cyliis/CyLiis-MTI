package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.Nodes.AutoStangaSouthNodes;
import org.firstinspires.ftc.teamcode.CV.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.DriveTrainControlTriggers;
import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.Standard;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Disabled
@Autonomous(name="Auto stanga south👉👌")
public class AutoStangaSouth extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    SampleMecanumDrive driveTrain;
    RobotModules robotModules;
    TiedBehaviour tiedBehaviour;

    Servo odo;

    double lastTime = -1;

    AprilTagDetector detector;

    AutoStangaSouthNodes nodes;

    public void initialize(){
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        robotModules = new RobotModules(hardwareMap, true);
        tiedBehaviour = new TiedBehaviour(robotModules);

        driveTrain = new SampleMecanumDrive(hardwareMap);

        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(0.7);

        nanoClock = NanoClock.system();

        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        detector = new AprilTagDetector(hardwareMap, telemetry);

        nodes = new AutoStangaSouthNodes(driveTrain, robotModules);

        nodes.initNodesAndTrajctories();
    }

    @Override
    public void runOpMode()  {
        initialize();

        while(!opModeIsActive() && !isStopRequested()){
            detector.loop();
        }

        detector.closeCamera();

        waitForStart();

        SampleMecanumDrive.imu.startIMUThread(this);

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        robotModules.atStart();

        while(opModeIsActive() && !isStopRequested()) {
            double timeMs = nanoClock.seconds()*1000;

            for(LynxModule hub:hubs)
                hub.clearBulkCache();

            nodes.current.run();
            driveTrain.update();

            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , 1000.0/(nanoClock.seconds()*1000 - timeMs));
            telemetry.addData("Loops/sec v2" , 1.0/(nanoClock.seconds() - lastTime));
            lastTime = nanoClock.seconds();
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        driveTrain.setDrivePower(new Pose2d());

        robotModules.emergencyStop();
    }
}