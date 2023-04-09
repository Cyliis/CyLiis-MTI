package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.CV.AprilTagDetector;
import org.firstinspires.ftc.teamcode.GamepadControl;
import org.firstinspires.ftc.teamcode.Modules.Claw;
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


@Autonomous(name="Auto Stanga Ok👉👌")
public class AutoStangaOk extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    SampleMecanumDrive driveTrain;
    RobotModules robotModules;
    TiedBehaviour tiedBehaviour;

    Servo odo;

    AprilTagDetector detector;

    public void initialize(){

        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        driveTrain = new SampleMecanumDrive(hardwareMap);
        robotModules = new RobotModules(hardwareMap, true);
        tiedBehaviour = new TiedBehaviour(robotModules);

        detector = new AprilTagDetector(hardwareMap, telemetry);

        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(0);

        nanoClock = NanoClock.system();

        driveTrain.followTrajectorySequenceAsync(coneTrajectory(index));
        index++;
    }


    Pose2d[] conePickup = {new Pose2d(49.2,23,1.5*Math.PI),
            new Pose2d(48.5,23,1.5*Math.PI),
            new Pose2d(48,23.5,1.5*Math.PI),
            new Pose2d(47.5,23.5,1.5*Math.PI),
            new Pose2d(47,24.8,1.5*Math.PI)
    };

    Pose2d[] coneRelease = {new Pose2d(52.5, -16, 5.851),
            new Pose2d(52, -16, 5.851),
            new Pose2d(51.5, -16, 5.851),
            new Pose2d(51, -17, 5.851),
            new Pose2d(50.5, -18, 5.851),
            new Pose2d(50, -19, 5.851)
    };

    TrajectorySequence preloadTrajectory(){
        return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(30,-6.5,0))
                .lineToLinearHeading(coneRelease[0])
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .build();
    }

    TrajectorySequence coneTrajectory(int index){
        if(index == 0) return preloadTrajectory();
        return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                .lineToLinearHeading(conePickup[index-1])
                .waitSeconds(0.1)
                .addTemporalMarker(()->robotModules.intake.setState(Intake.State.MCLOSING))
                .waitSeconds(0.1)
                .lineToLinearHeading(coneRelease[index])
                .UNSTABLE_addTemporalMarkerOffset(-1.5,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .build();
    }

    AprilTagDetector.DetectionResult result = AprilTagDetector.DetectionResult.Middle;

    TrajectorySequence parkingTrajectory(){
        switch (result){
            case Middle:
                return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(42,-3,0))
                        .build();
            case Left:
                return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(42,-3,0))
                        .strafeLeft(26)
                        .build();
            case Right:
                return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(42,-3,0))
                        .strafeRight(26)
                        .build();
        }
        return driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate()).build();
    }

    int index = 0;
    public static int conesFromStack = 5;

    public static double upWaitTime = .5111  ;

    boolean jammed = false;

    @Override
    public void runOpMode()  {
        initialize();
        while(!opModeIsActive() && !isStopRequested()){
            detector.loop();
            if(detector.getResult() != AprilTagDetector.DetectionResult.UNKNOWN && detector.getResult() != null) result = detector.getResult();
            telemetry.update();
        }

        waitForStart();

        detector.closeCamera();

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        robotModules.atStart();

        while(opModeIsActive() && !isStopRequested()) {
            double timeMs = nanoClock.seconds()*1000;

            for(LynxModule hub:hubs)
                hub.clearBulkCache();

            if(robotModules.intake.transferState == Intake.TransferState.ABORT && !jammed){
                jammed = true;
                index = conesFromStack + 2;
                robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                driveTrain.followTrajectorySequenceAsync(parkingTrajectory());
            }

            if(!driveTrain.isBusy()){
                if(index<=conesFromStack){
                    if(nanoClock.seconds() - robotModules.outtake.lift.timeOfLastStateChange >= upWaitTime) robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                    if(robotModules.outtake.state == Outtake.State.GOING_DOWN || robotModules.outtake.state == Outtake.State.DOWN){
                        Virtual.stackIndex = 5 - index;
                        driveTrain.followTrajectorySequenceAsync(coneTrajectory(index));
                        index ++;
                    }
                }
                else if(index == conesFromStack+1) {
                    if(nanoClock.seconds() - robotModules.outtake.lift.timeOfLastStateChange >= upWaitTime) robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                    if(robotModules.outtake.state == Outtake.State.GOING_DOWN || robotModules.outtake.state == Outtake.State.DOWN){
                        driveTrain.followTrajectorySequenceAsync(parkingTrajectory());
                        index ++;
                    }
                }
            }


            driveTrain.update();
            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , (int)(1000)/(nanoClock.seconds()*1000 - timeMs));
            telemetry.addData("Detectie", result);
             telemetry.addData("Trajectory index", index);
            telemetry.addData("Pose", driveTrain.getPoseEstimate());
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        robotModules.emergencyStop();
    }
}