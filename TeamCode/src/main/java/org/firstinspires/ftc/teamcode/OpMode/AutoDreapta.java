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


@Autonomous(name="Auto Dreapta👉👌")
public class AutoDreapta extends LinearOpMode {
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

        driveTrain.followTrajectorySequenceAsync(mainTrajectory());
    }

    boolean parked = false;

    Pose2d[] conePickup = {new Pose2d(50,24,1.5*Math.PI),
            new Pose2d(48,23,1.5*Math.PI),
            new Pose2d(48,24,1.5*Math.PI),
            new Pose2d(47,26,1.5*Math.PI),
            new Pose2d(46,27,1.5*Math.PI)
    };

    Pose2d[] coneRelease = {new Pose2d(54, -13, 5.692),
            new Pose2d(49, -12, 5.692),
            new Pose2d(49, -12, 5.692),
            new Pose2d(49, -12, 5.692),
            new Pose2d(48, -12, 5.692),
            new Pose2d(47, -11, 5.692)
    };

    TrajectorySequence mainTrajectory(){
        return driveTrain.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(30,-6.5,0))
                .lineToLinearHeading(coneRelease[0])
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                    robotModules.intake.setState(Intake.State.OPENING);
                    Virtual.stackIndex = 4;
                })
                .lineToLinearHeading(conePickup[0])
                .addTemporalMarker(()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .waitSeconds(0.2)
                .lineToLinearHeading(coneRelease[1])
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                    robotModules.intake.setState(Intake.State.OPENING);
                    Virtual.stackIndex = 3;
                })
                .lineToLinearHeading(conePickup[1])
                .addTemporalMarker(()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .waitSeconds(0.2)
                .lineToLinearHeading(coneRelease[2])
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                    robotModules.intake.setState(Intake.State.OPENING);
                    Virtual.stackIndex = 2;
                })
                .lineToLinearHeading(conePickup[2])
                .addTemporalMarker(()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .waitSeconds(0.2)
                .lineToLinearHeading(coneRelease[3])
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                    robotModules.intake.setState(Intake.State.OPENING);
                    Virtual.stackIndex = 1;
                })
                .lineToLinearHeading(conePickup[3])
                .addTemporalMarker(()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .waitSeconds(0.2)
                .lineToLinearHeading(coneRelease[4])
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                    robotModules.intake.setState(Intake.State.OPENING);
                    Virtual.stackIndex = 0;
                })
                .lineToLinearHeading(conePickup[4])
                .addTemporalMarker(()->robotModules.intake.setState(Intake.State.CLOSING))
                .waitSeconds(0.15)
                .forward(12)
                .addTemporalMarker(()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .waitSeconds(0.7)
                .lineToLinearHeading(coneRelease[5])
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robotModules.outtake.setState(Outtake.State.GOING_DOWN);
                    robotModules.intake.setState(Intake.State.OPENING);
                    Virtual.stackIndex = 0;
                })
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

            if(!driveTrain.isBusy() && !parked){
                driveTrain.followTrajectorySequenceAsync(parkingTrajectory());
                parked = true;
            }
//            if(!driveTrain.isBusy() && parked) break;


            driveTrain.update();
            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , (int)(1000)/(nanoClock.seconds()*1000 - timeMs));
            telemetry.addData("Detectie", result);
            telemetry.addData("Pose", driveTrain.getPoseEstimate());
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        robotModules.emergencyStop();
    }
}