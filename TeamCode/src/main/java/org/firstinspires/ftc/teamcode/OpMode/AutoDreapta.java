package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.CV.AprilTagDetector;
import org.firstinspires.ftc.teamcode.GamepadControl;
import org.firstinspires.ftc.teamcode.Modules.DriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

import javax.xml.parsers.FactoryConfigurationError;


@Autonomous(name="Auto Dreapta👉👌")
public class AutoDreapta extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    SampleMecanumDrive driveTrain;
    RobotModules robotModules;
    TiedBehaviour tiedBehaviour;

//    AprilTagDetector detector;

    public void initialize(){
        PhotonCore.enable();

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        driveTrain = new SampleMecanumDrive(hardwareMap);
        robotModules = new RobotModules(hardwareMap, true);
        tiedBehaviour = new TiedBehaviour(robotModules);

//        detector = new AprilTagDetector(hardwareMap, telemetry);

        nanoClock = NanoClock.system();

        driveTrain.followTrajectorySequenceAsync(mainTrajectory());
    }

    boolean parked = false;

    Pose2d[] conePickup = {new Pose2d(49.5,-23,0.5*Math.PI),
            new Pose2d(48,-22,0.5*Math.PI),
            new Pose2d(47,-22,0.5*Math.PI),
            new Pose2d(47,-22,0.5*Math.PI),
            new Pose2d(47,-26,0.5*Math.PI)
    };

    Pose2d[] coneRelease = {new Pose2d(58, 9, -5.373),
            new Pose2d(55, 10, -5.373),
            new Pose2d(54, 10, -5.373),
            new Pose2d(54, 10, -5.373),
            new Pose2d(54, 10, -5.373),
            new Pose2d(54, 10, -5.373)
    };

    TrajectorySequence mainTrajectory(){
        return driveTrain.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(16, 6.5, 0))
                .build();
    }

    AprilTagDetector.DetectionResult result = AprilTagDetector.DetectionResult.Middle;

    TrajectorySequence parkingTrajectory(){
        switch (result){
            case Middle:
                return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(45,6,0))
                        .build();
            case Left:
                return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(45,6,0))
                        .strafeLeft(26)
                        .build();
            case Right:
                return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(45,6,0))
                        .strafeRight(26)
                        .build();
        }
        return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate()).build();
    }

    @Override
    public void runOpMode()  {
        initialize();
        while(!opModeIsActive() && !isStopRequested()){
//            detector.loop();
//            if(detector.getResult() != AprilTagDetector.DetectionResult.UNKNOWN && detector.getResult() != null) result = detector.getResult();
            telemetry.update();
        }

        waitForStart();

//        detector.closeCamera();

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        robotModules.atStart();

        while(opModeIsActive() && !isStopRequested()) {
            double timeMs = nanoClock.seconds()*1000;

            for(LynxModule hub:hubs)
                hub.clearBulkCache();

            if(!driveTrain.isBusy() && !parked){
                driveTrain.followTrajectorySequenceAsync(parkingTrajectory());
                parked = true;
            }
            if(!driveTrain.isBusy() && parked) break;


            driveTrain.update();
            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , (int)(1000)/(nanoClock.seconds()*1000 - timeMs));
            telemetry.addData("Detectie", result);
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        robotModules.emergencyStop();
    }
}