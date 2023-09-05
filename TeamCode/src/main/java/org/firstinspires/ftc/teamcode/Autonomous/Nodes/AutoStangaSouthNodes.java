package org.firstinspires.ftc.teamcode.Autonomous.Nodes;

import com.acmerobotics.roadrunner.util.NanoClock;

import org.ejml.ops.MatrixFeatures_F;
import org.firstinspires.ftc.teamcode.Autonomous.Sussy.LogicNode;
import org.firstinspires.ftc.teamcode.Autonomous.Trajectories.AutoStangaSouthTrajectories;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Virtual;
import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoStangaSouthNodes {
    LogicNode start;
    LogicNode preload;
    LogicNode afterPreload;
    LogicNode pickupStack;
    LogicNode releaseStackCone;
    LogicNode waitForLift;
    LogicNode end;

    SampleMecanumDrive drive;
    RobotModules robot;

    AutoStangaSouthTrajectories trajectories;

    NanoClock nanoClock;

    private int coneIndex;

    public LogicNode current;

    public AutoStangaSouthNodes(SampleMecanumDrive drive, RobotModules robot){
        this.drive = drive;
        this.robot = robot;
        nanoClock = NanoClock.system();
        trajectories = new AutoStangaSouthTrajectories(drive, robot);
        coneIndex = 4;
    }

    public void initNodesAndTrajctories(){
        trajectories.initTrajectories();

        start = new LogicNode();
        preload = new LogicNode();
        waitForLift = new LogicNode();
        afterPreload = new LogicNode();
        pickupStack = new LogicNode();
        releaseStackCone = new LogicNode();
        end = new LogicNode();
        current = new LogicNode();

        start.addCondition(()->true, ()->drive.followTrajectorySequenceAsync(trajectories.preloadRelease), preload);
        preload.addCondition(()->!drive.isBusy(), ()->{}, waitForLift);
        waitForLift.addCondition(()->coneIndex == 4 && robot.outtake.state == Outtake.State.HIGH && nanoClock.seconds() - robot.outtake.timeOfLastStateChange >= 0,
                ()-> {
                    robot.outtake.setState(Outtake.State.GOING_DOWN);
                    drive.followTrajectorySequenceAsync(trajectories.afterPreload);
                }, afterPreload);
        afterPreload.addCondition(()->!drive.isBusy(), ()-> {
            drive.followTrajectorySequenceAsync(trajectories.stackConePickup.get(coneIndex));
            Virtual.stackIndex = coneIndex;
        }, pickupStack);
        waitForLift.addCondition(()->coneIndex == -1 && robot.outtake.state == Outtake.State.HIGH && nanoClock.seconds() - robot.outtake.timeOfLastStateChange >= 0,
                ()->robot.outtake.setState(Outtake.State.GOING_DOWN), end);
        waitForLift.addCondition(()->robot.outtake.state == Outtake.State.HIGH && nanoClock.seconds() - robot.outtake.timeOfLastStateChange >= 0,
                ()->{
            drive.followTrajectorySequenceAsync(trajectories.stackConePickup.get(coneIndex));
                    Virtual.stackIndex = coneIndex;
                    robot.outtake.setState(Outtake.State.GOING_DOWN);}, pickupStack);
        pickupStack.addCondition(()->!drive.isBusy(),
                ()-> {
                    drive.followTrajectorySequenceAsync(trajectories.stackConeRelease.get(coneIndex));
                    coneIndex--;
                }, releaseStackCone);
        releaseStackCone.addCondition(()->!drive.isBusy(), ()->{}, waitForLift);
        current = start;
    }

}
