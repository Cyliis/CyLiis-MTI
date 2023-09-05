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
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Latch;
import org.firstinspires.ftc.teamcode.Modules.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name="Auto dreapta contested👉👌")
public class AutoDreaptaContested extends LinearOpMode {
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
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
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
    }

    @Override
    public void runOpMode()  {
        initialize();

        double OFFSET = 2.7;
        double PICK_UP_X = 47;
        double PICK_UP_Y = -21;
        double[] PICK_UP_Y_OFFSET = {-0.3,0.4,0.6,0.5,0.6};

        TrajectorySequence bruh1 = driveTrain.trajectorySequenceBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(15,5.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .splineToSplineHeading(new Pose2d(52,11,-5.8),-6)
                .waitSeconds(0.1)
                .addTemporalMarker(()->robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .addTemporalMarker(()-> Virtual.stackIndex=4)
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 0*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 1*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 1*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 2*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 2*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 3*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 3*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 4*OFFSET, PICK_UP_Y - 1, PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 4*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .back(6)
                .lineToLinearHeading(new Pose2d(36, 5, 0))
                .lineToLinearHeading(new Pose2d(36, 29,0))
                .build();

        TrajectorySequence bruh2 = driveTrain.trajectorySequenceBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(15,5.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .splineToSplineHeading(new Pose2d(52,11,-5.8),-6)
                .waitSeconds(0.1)
                .addTemporalMarker(()->robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .addTemporalMarker(()-> Virtual.stackIndex=4)
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 0*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 1*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 1*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 2*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 2*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 3*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 3*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 4*OFFSET, PICK_UP_Y - 1, PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 4*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .back(6)
                .lineToLinearHeading(new Pose2d(36, 5, 0))
                .build();

        TrajectorySequence bruh3 = driveTrain.trajectorySequenceBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(15,5.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .splineToSplineHeading(new Pose2d(52,11,-5.8),-6)
                .waitSeconds(0.1)
                .addTemporalMarker(()->robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .addTemporalMarker(()-> Virtual.stackIndex=4)
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 0*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 1*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 1*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 2*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 2*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 3*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 3*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 4*OFFSET, PICK_UP_Y - 1, PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 4*OFFSET, 11, -5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .back(6)
                .lineToLinearHeading(new Pose2d(36, 5, 0))
                .lineToLinearHeading(new Pose2d(36, -17 ,0))
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
            telemetry.addData("Result", detector.getResult());
            lastTime = nanoClock.seconds();
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        driveTrain.setDrivePower(new Pose2d());

        robotModules.emergencyStop();
    }
}