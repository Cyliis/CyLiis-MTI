package org.firstinspires.ftc.teamcode.Autonomous.Sussy;

import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class SussyNode {

    SampleMecanumDrive dt;
    RobotModules robot;

    public interface AutoCondition{
        boolean autoCondition();
    }

    private class AutoIf {
        AutoCondition condition;
        Runnable preTransition;
        SussyNode transitionNode;

        public AutoIf(AutoCondition condition, Runnable preTransition, SussyNode transitionNode){
            this.condition = condition;
            this.preTransition = preTransition;
            this.transitionNode = transitionNode;
        }
    }

    public ArrayList <AutoIf> conditionsList;

    public SussyNode(SampleMecanumDrive dt, RobotModules robot){
        this.dt = dt;
        this.robot = robot;

    }

    public void addCondition(AutoCondition condition, Runnable preTransition, SussyNode transitionNode){
        conditionsList.add(new AutoIf(condition, preTransition, transitionNode));
    }

    private void transition(SussyNode transitionNode){
        conditionsList = transitionNode.conditionsList;
    }

    public void run(){
        for(AutoIf autoIf:conditionsList){
            if(autoIf.condition.autoCondition()){
                autoIf.preTransition.run();
                transition(autoIf.transitionNode);
                break;
            }
        }
    }

}
