package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;

import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.Trajectory;

@Config
public class Follower implements IRobotModule {

    private Trajectory trajectory;
    private final MecanumDrive mecanumDrive;
    private final Localizer localizer;

    public Follower(MecanumDrive mecanumDrive, Localizer localizer){
        this.mecanumDrive = mecanumDrive;
        this.localizer = localizer;
        this.trajectory = new Trajectory(new Pose(localizer.getPoseEstimate()));

        mecanumDrive.setRunMode(MecanumDrive.RunMode.PID);
    }

    public void setTrajectory(Trajectory trajectory){
        this.trajectory = trajectory;
    }

    public Trajectory getTrajectory(){
        return trajectory;
    }

    @Override
    public void loop() {
        if(trajectory.getCurrentPoint().isReached(new Pose(localizer.getPoseEstimate()))){
            mecanumDrive.setTargetPose(trajectory.getNextPoint());
        }
    }

    @Override
    public void atStart() {

    }

    @Override
    public void emergencyStop() {

    }
}
