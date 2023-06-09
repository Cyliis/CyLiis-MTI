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


@Autonomous(name="Auto stanga mid👉👌")
public class AutoStangaMid extends LinearOpMode {
    List<LynxModule> hubs;
    FtcDashboard dash;
    NanoClock nanoClock;

    SampleMecanumDrive driveTrain;
    RobotModules robotModules;
    TiedBehaviour tiedBehaviour;

    Servo odo;

    public void initialize(){
        dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry,dash.getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub:hubs) {
            if(hub.getImuType() == LynxModuleImuType.BHI260) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            else hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }

        robotModules = new RobotModules(hardwareMap, true);
        tiedBehaviour = new TiedBehaviour(robotModules);

        driveTrain = new SampleMecanumDrive(hardwareMap);

        odo = hardwareMap.get(Servo.class, "odo");
        odo.setPosition(0.2);

        nanoClock = NanoClock.system();

        PhotonCore.enable();
    }

    @Override
    public void runOpMode()  {
        initialize();

        double OFFSET = -1.9;
        double PICK_UP_X = 49.5;
        double PICK_UP_Y = 22;
        double[] PICK_UP_Y_OFFSET = {0,-0.4,-0.6,-0.5,-0.6};

        TrajectorySequence bruh = driveTrain.trajectorySequenceBuilder(new Pose2d())
                .UNSTABLE_addTemporalMarkerOffset(0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .splineToSplineHeading(new Pose2d(30,-11,5.75),5.75)
                .addTemporalMarker(()->robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .addTemporalMarker(()-> Virtual.stackIndex=4)
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, -5, -PI/2))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 0*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(45 - 0*OFFSET, -10, 3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 1*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(45 - 1*OFFSET, -10, 3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 2*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(45 - 2*OFFSET, -10, 3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 3*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(45 - 3*OFFSET, -11, 3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .lineToLinearHeading(new Pose2d(PICK_UP_X - 4*OFFSET, PICK_UP_Y + PICK_UP_Y_OFFSET[Virtual.stackIndex], -PI/2))
                .addTemporalMarker(()->robotModules.intake.claw.setState(Claw.State.MCLOSING))
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->robotModules.intake.setState(Intake.State.TRANSFERING))
                .lineToLinearHeading(new Pose2d(45 - 4*OFFSET, -11, 3.8))
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> robotModules.outtake.setState(Outtake.State.GOING_MID))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> robotModules.outtake.setState(Outtake.State.GOING_DOWN))
                .waitSeconds(0.1)
                .back(5)
                .lineToLinearHeading(new Pose2d(58, -26, 0))
                .build();

        waitForStart();

        driveTrain.followTrajectorySequenceAsync(bruh);

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

            telemetry.addData("Loops/sec" , (int)(1000)/(nanoClock.seconds()*1000 - timeMs));
            robotModules.telemetry(telemetry);

            telemetry.update();
        }

        driveTrain.setDrivePower(new Pose2d());

        robotModules.emergencyStop();
    }
}