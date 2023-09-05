package org.firstinspires.ftc.teamcode.Utils;

import java.util.ArrayList;

public class Trajectory {
    private ArrayList<Pose> points = new ArrayList<>();

    private int index = 0;
    private int size = 1;

    public Trajectory(Pose startingPose){
        points.add(startingPose);
    }

    public Trajectory addPoint(Pose pose){
        size++;
        points.add(pose);
        return this;
    }

    public Pose getNextPoint(){
        if(index + 1 == size) return points.get(index);
        index ++;
        return points.get(index);
    }

    public Pose getCurrentPoint(){
        return points.get(index);
    }
}
