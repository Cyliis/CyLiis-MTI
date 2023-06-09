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

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.TiedBehaviour;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;


@Autonomous(name="Auto stanga contested👉👌")
public class AutoStangaContested extends LinearOpMode {
    FtcDashboard dash;
    NanoClock nanoClock;

    SampleMecanumDrive driveTrain;
    RobotModules robotModules;
    TiedBehaviour tiedBehaviour;

    Servo odo;

    public void initialize(){
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());

        robotModules = new RobotModules(hardwareMap, true);
        tiedBehaviour = new TiedBehaviour(robotModules);

        driveTrain = new SampleMecanumDrive(hardwareMap);

        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(0.2);

        nanoClock = NanoClock.system();

        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
    }

    @Override
    public void runOpMode()  {
        initialize();

        double OFFSET = 1.7;
        double PICK_UP_X = 48.5;
        double PICK_UP_Y = 21;
        double[] PICK_UP_Y_OFFSET = {0,-0.4,-0.6,-0.5,-0.6};

        TrajectorySequence bruh = driveTrain.trajectorySequenceBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(15,-5.5, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(0.5,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .splineToSplineHeading(new Pose2d(52,-12,5.8),6)
                .waitSeconds(0.1)
                .addTemporalMarker(()->robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .addTemporalMarker(()-> Virtual.stackIndex=4)
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 0*OFFSET, -13, 5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 1*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 1*OFFSET, -13, 5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 2*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 2*OFFSET, -13, 5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 3*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 3*OFFSET, -13, 5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 4*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(49 - 4*OFFSET, -13, 5.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.4,() -> robotModules.outtake.setState(Outtake.State.GOING_HIGH))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(17, -5, 0))
                .lineToLinearHeading(new Pose2d(17, -29,0))
                .build();

        waitForStart();

        driveTrain.followTrajectorySequenceAsync(bruh);

        SampleMecanumDrive.imu.startIMUThread(this);

        robotModules.atStart();

        while(opModeIsActive() && !isStopRequested()) {
            double timeMs = nanoClock.seconds()*1000;

            driveTrain.update();

            robotModules.loop();
            tiedBehaviour.loop();

            telemetry.addData("Loops/sec" , (int)(1000)/(nanoClock.seconds()*1000 - timeMs));
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        driveTrain.setDrivePower(new Pose2d());

        robotModules.emergencyStop();
    }
}