package org.firstinspires.ftc.teamcode.OpMode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CV.AprilTagDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Park👉👌")
public class Parcare extends LinearOpMode {

    SampleMecanumDrive drive;
    AprilTagDetector detector;

    AprilTagDetector.DetectionResult result = AprilTagDetector.DetectionResult.Middle;

    TrajectorySequence traj(){

        switch (result){
            case Middle:
                return drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(26)
                        .build();
            case Left:
                return drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(26)
                        .strafeLeft(26)
                        .build();
            case Right:
                return drive.trajectorySequenceBuilder(new Pose2d())
                        .forward(26)
                        .strafeRight(26)
                        .build();
        }

        return drive.trajectorySequenceBuilder(new Pose2d())
                .forward(32)
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        detector = new AprilTagDetector(hardwareMap, telemetry);

        while(opModeInInit() && !isStopRequested()){
            detector.loop();
            if(detector.getResult()!= AprilTagDetector.DetectionResult.UNKNOWN) result = detector.getResult();
            telemetry.addData("result", result);
        }

        waitForStart();

        detector.closeCamera();

        drive.followTrajectorySequenceAsync(traj());

        while(opModeIsActive() && !isStopRequested()){
            drive.update();
            if(!drive.isBusy()) break;
        }
    }
}
