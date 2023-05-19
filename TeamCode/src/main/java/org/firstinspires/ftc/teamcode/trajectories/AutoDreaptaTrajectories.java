package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutoDreaptaTrajectories {
    private SampleMecanumDrive drive;
    private RobotModules robot;

    public AutoDreaptaTrajectories(SampleMecanumDrive drive, RobotModules robot){
        this.drive = drive;
        this.robot = robot;
    }

    public static double liftTemporalOffset = -0.7;

    public static double preloadX = 34, preloadY = -32.5, preloadHeading = 5.23;

    private Pose2d preloadPosition = new Pose2d(preloadX, preloadY, preloadHeading);

    public TrajectorySequence preloadTrajectory(){
        return drive.trajectorySequenceBuilder(new Pose2d())
                .splineToSplineHeading(preloadPosition, preloadPosition.getHeading())
                .UNSTABLE_addTemporalMarkerOffset(liftTemporalOffset, ()->robot.outtake.setState(Outtake.State.HIGH))
                .waitSeconds(0.05)
                .addTemporalMarker(()->robot.outtake.setState(Outtake.State.GOING_DOWN))
                .addTemporalMarker(()->drive.followTrajectorySequenceAsync(afterPreloadTrajectory()))
                .build();
    }

    public static double aPreloadX = 50, aPreloadY = -25, aPreloadHeading = 1.5*Math.PI;

    private Pose2d afterPreloadPosition = new Pose2d(aPreloadX, aPreloadY, aPreloadHeading);
    
    public TrajectorySequence afterPreloadTrajectory(){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(afterPreloadPosition)
                .build();
    }
    
    public static double firstConePickupX = 52, firstConePickupY = 21.5, firstConePickupH = 1.5*Math.PI;
    public static double secondConePickupX = 52, secondConePickupY = 21.5,secondConePickupH = 1.5*Math.PI;
    public static double thirdConePickupX = 52, thirdConePickupY = 21.5, thirdConePickupH = 1.5*Math.PI;
    public static double fourthConePickupX = 52, fourthConePickupY = 21.5, fourthConePickupH = 1.5*Math.PI;
    public static double fifthConePickupX = 52, fifthConePickupY = 23.3, fifthConePickupH = 1.5*Math.PI;
    
    private Pose2d[] pickupTrajectories = {
            new Pose2d(firstConePickupX, firstConePickupY, firstConePickupH),
            new Pose2d(secondConePickupX, secondConePickupY,secondConePickupH),
            new Pose2d(thirdConePickupX, thirdConePickupY, thirdConePickupH),
            new Pose2d(fourthConePickupX, fourthConePickupY, fourthConePickupH),
            new Pose2d(fifthConePickupX, fifthConePickupY, fifthConePickupH)
    };
    
    public TrajectorySequence pickupTrajectory(int index){
        Virtual.stackIndex = 4 - index;
        robot.intake.setState(Intake.State.OPENING);
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .lineToSplineHeading(pickupTrajectories[index])
                .waitSeconds(0.05)
                .addTemporalMarker(()->robot.intake.setState(Intake.State.TRANSFERING))
                .waitSeconds(0.05)
                .build();
    }

    public static double firstConeReleaseX=43, firstConeReleaseY=-35, firstConeReleaseH=4.058;
    public static double secondConeReleaseX=43, secondConeReleaseY=-35,secondConeReleaseH=4.058;
    public static double thirdConeReleaseX=43, thirdConeReleaseY=-35, thirdConeReleaseH=4.058;
    public static double fourthConeReleaseX=43, fourthConeReleaseY=-35, fourthConeReleaseH=4.058;
    public static double fifthConeReleaseX=43, fifthConeReleaseY=-35, fifthConeReleaseH=4.058;

    private Pose2d[] releaseTrajectories = {
            new Pose2d(firstConeReleaseX, firstConeReleaseY, firstConeReleaseH),
            new Pose2d(secondConeReleaseX, secondConeReleaseY,secondConeReleaseH),
            new Pose2d(thirdConeReleaseX, thirdConeReleaseY, thirdConeReleaseH),
            new Pose2d(fourthConeReleaseX, fourthConeReleaseY, fourthConeReleaseH),
            new Pose2d(fifthConeReleaseX, fifthConeReleaseY, fifthConeReleaseH)
    };

    public static double firstConeForwardX = 52, firstConeForwardY = -10, firstConeForwardH = 1.5*Math.PI;
    public static double secondConeForwardX = 52, secondConeForwardY= -10,secondConeForwardH = 1.5*Math.PI;
    public static double thirdConeForwardX = 52, thirdConeForwardY= -10, thirdConeForwardH = 1.5*Math.PI;
    public static double fourthConeForwardX = 52, fourthConeForwardY= -10, fourthConeForwardH = 1.5*Math.PI;
    public static double fifthConeForwardX = 52, fifthConeForwardY= -10, fifthConeForwardH = 1.5*Math.PI;

    private Pose2d[] forwardTrajectories = {
            new Pose2d(firstConeForwardX, firstConeForwardY, firstConeForwardH),
            new Pose2d(secondConeForwardX, secondConeForwardY,secondConeForwardH),
            new Pose2d(thirdConeForwardX, thirdConeForwardY, thirdConeForwardH),
            new Pose2d(fourthConeForwardX, fourthConeForwardY, fourthConeForwardH),
            new Pose2d(fifthConeForwardX, fifthConeForwardY, fifthConeForwardH)
    };

    public TrajectorySequence releaseTrajectory(int index){
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(forwardTrajectories[index])
                .splineToSplineHeading(releaseTrajectories[index], releaseTrajectories[index].getHeading())
                .waitSeconds(0.05)
                .build();
    }
}
