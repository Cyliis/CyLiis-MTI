package org.firstinspires.ftc.teamcode.Utils;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class Pose {
    private final double x;
    private final double y;
    private double heading;
    private final double tolerance;
    public static double defaultTolerance = 0.5;

    public Pose(double x, double y, double heading, double tolerance){
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.tolerance = tolerance;
    }

    public Pose(Pose2d otherPose){
        this(otherPose.getX(), otherPose.getY(), otherPose.getHeading(), defaultTolerance);
    }

    public Pose(Pose2d otherPose, double tolerance){
        this(otherPose.getX(), otherPose.getY(), otherPose.getHeading(), tolerance);
    }

    public Pose(double x, double y, double heading){
        this(x, y, heading,defaultTolerance);
    }

    public Pose(){
        this(0,0,0,defaultTolerance);
    }

    public double getX(){
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public double getTolerance(){
        return tolerance;
    }

    public double getDistance(Pose other){
        return Math.sqrt((other.getX() - x)*(other.getX() - x) + (other.getY() - y)*(other.getY() - y));
    }

    public boolean isReached(Pose other){
        return getDistance(other) <= tolerance;
    }

    public boolean isHeadingReached(Pose other, double headingTolerance){
        double angleDelta = other.getHeading() - heading;
        while(heading < -PI) heading += 2.0*PI;
        while(heading > PI) heading -= 2.0*PI;
        return Math.abs(angleDelta) <= headingTolerance;
    }
}
