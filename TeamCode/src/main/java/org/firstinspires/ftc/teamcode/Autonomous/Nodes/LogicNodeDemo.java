package org.firstinspires.ftc.teamcode.Autonomous.Nodes;


import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Autonomous.Sussy.LogicNode;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class LogicNodeDemo {
    TrajectorySequence sussyTrajectory;

    public LogicNodeDemo(SampleMecanumDrive dt, RobotModules robot){
        sussyTrajectory = dt.trajectorySequenceBuilder(new Pose2d()).build();
        initNodes(dt, robot);
    }

    private final double velocityThreshold = 0.1;

    public LogicNode currentNode;

    LogicNode start;
    LogicNode goToPreload;
    LogicNode goBackToRam;
    LogicNode goPickCone;

    private void initNodes(SampleMecanumDrive dt, RobotModules robotModules){
        start = new LogicNode(dt, robotModules);
        start.addCondition(()->true, ()->{dt.followTrajectorySequenceAsync(sussyTrajectory);}, goToPreload);
        goToPreload = new LogicNode(dt, robotModules);
        goToPreload.addCondition(()->dt.getWheelVelocities().get(0) < velocityThreshold && dt.getWheelVelocities().get(1) < velocityThreshold, ()->dt.followTrajectorySequenceAsync(sussyTrajectory), goBackToRam);
        goToPreload.addCondition(()->!dt.isBusy() && robotModules.outtake.state == Outtake.State.HIGH, ()->robotModules.outtake.setState(Outtake.State.GOING_DOWN), goPickCone);


        currentNode = start;
    }

    public void update(){
        currentNode.run();
    }
}
