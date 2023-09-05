package org.firstinspires.ftc.teamcode.Autonomous.Nodes;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.apache.commons.math3.analysis.function.Log;
import org.firstinspires.ftc.teamcode.Autonomous.PIDTuningAuto;
import org.firstinspires.ftc.teamcode.Autonomous.Sussy.LogicNode;
import org.firstinspires.ftc.teamcode.Modules.Follower;
import org.firstinspires.ftc.teamcode.Modules.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Trajectory;

@Config
public class PIDTuningNodes {

    public static double tolerance = 0.5, headingTolerance = 0.1;

    private MecanumDrive drive;
    private Follower follower;

    public PIDTuningNodes(MecanumDrive drive, Follower follower){
        this.drive = drive;
        this.follower = follower;
        initNodes();
    }

    public LogicNode move = new LogicNode();
    public LogicNode turn = new LogicNode();
    public LogicNode current = new LogicNode();

    public Trajectory moveTrajectory = new Trajectory(new Pose(24,0,0,tolerance));
    public Trajectory turnTrajectory = new Trajectory(new Pose(0,0,PI/2.0,tolerance));

    private void initNodes(){
        move.addCondition(()->{
                    return follower.getTrajectory().getCurrentPoint().isReached(drive.getPoseEstimate());
                },()->{
            drive.getLocalizer().setPoseEstimate(new Pose2d());
            follower.setTrajectory(turnTrajectory);
                },turn);

        turn.addCondition(()->{
            return follower.getTrajectory().getCurrentPoint().isHeadingReached(drive.getPoseEstimate(), headingTolerance);
        }, ()->{
            drive.getLocalizer().setPoseEstimate(new Pose2d());
            follower.setTrajectory(moveTrajectory);
        }, move);

        current = move;
    }

}
