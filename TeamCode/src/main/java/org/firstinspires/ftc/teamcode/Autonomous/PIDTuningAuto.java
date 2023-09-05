package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomous.Nodes.AutoStangaSouthNodes;
import org.firstinspires.ftc.teamcode.Autonomous.Nodes.PIDTuningNodes;
import org.firstinspires.ftc.teamcode.CV.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.Follower;
import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.DriveTrainControlTriggers;
import org.firstinspires.ftc.teamcode.Modules.GamepadControllers.Standard;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;
import org.firstinspires.ftc.teamcode.Utils.ImuModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Trajectory;
import org.firstinspires.ftc.teamcode.drive.FunnyLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name="PID Tuning👉👌")
public class PIDTuningAuto extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    MecanumDrive driveTrain;
//    RobotModules robotModules;
//    TiedBehaviour tiedBehaviour;
    ImuModule imu;

    Localizer localizer;
    Follower follower;

    Servo odo;

    double lastTime = -1;

    AprilTagDetector detector;

    PIDTuningNodes nodes;

    public void initialize(){
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

//        robotModules = new RobotModules(hardwareMap, true);
//        tiedBehaviour = new TiedBehaviour(robotModules);

        imu = new ImuModule(hardwareMap);
        localizer = new FunnyLocalizer(hardwareMap, imu);

        driveTrain = new MecanumDrive(hardwareMap, localizer, MecanumDrive.RunMode.PID);

        follower = new Follower(driveTrain, localizer);

        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(0.7);

        nanoClock = NanoClock.system();

        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();

        detector = new AprilTagDetector(hardwareMap, telemetry);

        nodes = new PIDTuningNodes(driveTrain, follower);
    }

    @Override
    public void runOpMode()  {
        initialize();

        while(!opModeIsActive() && !isStopRequested()){
            detector.loop();
        }

        detector.closeCamera();

        waitForStart();

        imu.startIMUThread(this);

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

//        robotModules.atStart();

        Trajectory funny = new Trajectory(new Pose())
                .addPoint(new Pose(24,0,0))
                .addPoint(new Pose(24,0,PI/2.0));

        follower.setTrajectory(funny);

        while(opModeIsActive() && !isStopRequested()) {
            double timeMs = nanoClock.seconds()*1000;

            for(LynxModule hub:hubs)
                hub.clearBulkCache();

//            nodes.current.run();
            localizer.update();
            follower.loop();
            driveTrain.loop();

//            robotModules.loop();
//            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , 1000.0/(nanoClock.seconds()*1000 - timeMs));
            telemetry.addData("Loops/sec v2" , 1.0/(nanoClock.seconds() - lastTime));
            lastTime = nanoClock.seconds();
//            robotModules.telemetry(telemetry);

            telemetry.addData("output x", driveTrain.powerCoolVector.getX());
            telemetry.addData("output y", driveTrain.powerCoolVector.getY());
            telemetry.addData("output h", driveTrain.powerCoolVector.getZ());
            telemetry.addData("pos", driveTrain.getLocalizer().getPoseEstimate());
            telemetry.addData("target x", follower.getTrajectory().getCurrentPoint().getX());
            telemetry.addData("target y", follower.getTrajectory().getCurrentPoint().getY());
            telemetry.addData("target h", follower.getTrajectory().getCurrentPoint().getHeading());
            telemetry.update();

            telemetry.update();
        }

//        robotModules.emergencyStop();
    }
}