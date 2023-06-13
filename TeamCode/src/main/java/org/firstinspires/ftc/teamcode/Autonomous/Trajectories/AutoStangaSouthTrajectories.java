package org.firstinspires.ftc.teamcode.Autonomous.Trajectories;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Modules.Claw;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

import java.util.ArrayList;

public class AutoStangaSouthTrajectories {

    SampleMecanumDrive drive;
    RobotModules robot;

    public AutoStangaSouthTrajectories(SampleMecanumDrive drive, RobotModules robot){
        this.drive = drive;
        this.robot = robot;
    }

    public TrajectorySequence preloadRelease;
    public TrajectorySequence afterPreload;
    public ArrayList<TrajectorySequence> stackConeRelease= new ArrayList<>();
    public ArrayList<TrajectorySequence> stackConePickup = new ArrayList<>();

    public TrajectorySequence[] parkFromPickup;
    public TrajectorySequence[] parkFromRelease;

    private final double xOffset = 2, yOffset = 0;

    double[] PICK_UP_Y_OFFSET = {0,-0.4,-0.6,-0.5,-0.6};

    private double pickupX = 50.5, pickupY = 22, pickupH = -PI/2;

    private double releaseX = 43.5, releaseY = -34, releaseH = 4.06;

    private double preloadX = 34, preloadY = -35.5, preloadH = Math.toRadians(300);

    private double afterPreloadX = 50, afterPreloadY = -28, afterPreloadH = -PI/2;

    public void initTrajectories(){
        drive.setPoseEstimate(new Pose2d());
        preloadRelease = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(preloadX, preloadY, preloadH), preloadH)
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->robot.outtake.setState(Outtake.State.GOING_HIGH))
                .build();
        afterPreload = drive.trajectorySequenceBuilder(new Pose2d(preloadX, preloadY, preloadH))
                .back(5)
                .lineToLinearHeading(new Pose2d(afterPreloadX, afterPreloadY, afterPreloadH))
                .build();
        for(int i = 0; i < 5; i++){
            if(i==4) stackConePickup.add(drive.trajectorySequenceBuilder(new Pose2d(afterPreloadX, afterPreloadY, afterPreloadH))
                    .lineToLinearHeading(new Pose2d(pickupX, pickupY + PICK_UP_Y_OFFSET[i], pickupH))
                    .build());
            else stackConePickup.add(drive.trajectorySequenceBuilder(new Pose2d(releaseX + xOffset*(4 - (i+1)), releaseY + yOffset*(4 - (i+1)), releaseH))
                    .lineToLinearHeading(new Pose2d(pickupX + xOffset*i, pickupY + yOffset*i + PICK_UP_Y_OFFSET[i], pickupH))
                    .build());
        }
        for(int i = 0; i < 5; i++){
            stackConeRelease.add(drive.trajectorySequenceBuilder(new Pose2d(pickupX + (4-i)*xOffset, pickupY + (4-i)*yOffset, pickupH))
                    .addTemporalMarker(()->robot.intake.claw.setState(Claw.State.MCLOSING))
                    .UNSTABLE_addTemporalMarkerOffset(0.1,()->robot.intake.setState(Intake.State.TRANSFERING))
                    .splineToSplineHeading(new Pose2d(releaseX + xOffset* i, releaseY + yOffset* i, releaseH), 4.5)
                    .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->robot.outtake.setState(Outtake.State.GOING_HIGH))
                    .build());
        }

    }


}
