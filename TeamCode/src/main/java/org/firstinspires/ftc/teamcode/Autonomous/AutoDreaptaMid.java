package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LynxModuleImuType;
import com.qualcomm.robotcore.hardware.Servo;

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


@Autonomous(name="Auto Dreapta Mid👉👌")
public class AutoDreaptaMid extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    SampleMecanumDrive driveTrain;
    RobotModules robotModules;
    TiedBehaviour tiedBehaviour;

    Servo odo;

    double lastTime = -1;

    AprilTagDetector detector;

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

        detector = new AprilTagDetector(hardwareMap,telemetry);
    }

    @Override
    public void runOpMode()  {
        initialize();

        double OFFSET = -1.7;
        double PICK_UP_X = 48.2;
        double PICK_UP_Y = -22;
        double[] PICK_UP_Y_OFFSET = {0.2,0.4,0.6,0.5,0.6};

        TrajectorySequence bruh1 = driveTrain.trajectorySequenceBuilder(new Pose2d())
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .splineToSplineHeading(new Pose2d(29.5,11,-5.75),-5.75)
                .addTemporalMarker(()->robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .addTemporalMarker(()-> Virtual.stackIndex=4)
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, 5, PI/2))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(45 - 0*OFFSET, 10, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 1*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 1*OFFSET, 10.5 + 1, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 2*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 2*OFFSET, 10.5 + 1, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 3*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 3*OFFSET, 11, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 4*OFFSET, PICK_UP_Y - 1, PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 4*OFFSET, 11 + 1, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .back(5)
                .lineToLinearHeading(new Pose2d(54, 26, 0))
                .build();

        TrajectorySequence bruh2 = driveTrain.trajectorySequenceBuilder(new Pose2d())
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .splineToSplineHeading(new Pose2d(29.5,11,-5.75),-5.75)
                .addTemporalMarker(()->robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .addTemporalMarker(()-> Virtual.stackIndex=4)
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, 5, PI/2))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(45 - 0*OFFSET, 10, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 1*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 1*OFFSET, 10.5 + 1, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 2*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 2*OFFSET, 10.5 + 1, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 3*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 3*OFFSET, 11, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 4*OFFSET, PICK_UP_Y - 1, PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 4*OFFSET, 11 + 1, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .back(5)
                .lineToLinearHeading(new Pose2d(54, 2, 0))
                .build();

        TrajectorySequence bruh3 = driveTrain.trajectorySequenceBuilder(new Pose2d())
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .splineToSplineHeading(new Pose2d(29.5,11,-5.75),-5.75)
                .addTemporalMarker(()->robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .addTemporalMarker(()-> Virtual.stackIndex=4)
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, 5, PI/2))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(45 - 0*OFFSET, 10, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 1*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 1*OFFSET, 10.5 + 1, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 2*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 2*OFFSET, 10.5 + 1, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 3*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 3*OFFSET, 11, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 4*OFFSET, PICK_UP_Y - 1, PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(46 - 4*OFFSET, 11 + 1, -3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .back(5)
                .lineToLinearHeading(new Pose2d(54, -22, 0))
                .build();

        while (!opModeIsActive() && !isStopRequested()){
            detector.loop();
            telemetry.addData("Apriltag" ,detector.getResult());
            telemetry.update();
        }

        detector.closeCamera();

        waitForStart();

        switch (detector.getResult()){
            case Left:
                driveTrain.followTrajectorySequenceAsync(bruh1);
                break;
            case Right:
                driveTrain.followTrajectorySequenceAsync(bruh3);
                break;
            case Middle:
            case UNKNOWN:
                driveTrain.followTrajectorySequenceAsync(bruh2);
                break;
        }

        SampleMecanumDrive.imu.startIMUThread(this);

        for(LynxModule hub:hubs)
            hub.clearBulkCache();

        robotModules.atStart();

        while(opModeIsActive() && !isStopRequested()) {
            double timeMs = nanoClock.seconds()*1000;

            for(LynxModule hub:hubs)
                hub.clearBulkCache();

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