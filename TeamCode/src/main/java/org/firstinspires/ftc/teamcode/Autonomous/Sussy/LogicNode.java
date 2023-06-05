package org.firstinspires.ftc.teamcode.Autonomous.Sussy;

import org.firstinspires.ftc.teamcode.RobotModules;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class LogicNode {

    SampleMecanumDrive dt;
    RobotModules robot;

    public interface AutoCondition{
        boolean autoCondition();
    }

    private class AutoIf {
        AutoCondition condition;
        Runnable preTransitionMethod;
        LogicNode transitionNode;

        public AutoIf(AutoCondition condition, Runnable preTransition, LogicNode transitionNode){
            this.condition = condition;
            this.preTransitionMethod = preTransition;
            this.transitionNode = transitionNode;
        }
    }

    public void addCondition(AutoCondition condition, Runnable preTransitionMethod, LogicNode transitionNode){
        conditionsList.add(new AutoIf(condition, preTransitionMethod, transitionNode));
    }

    public void run(){
        for(AutoIf autoIf:conditionsList){
            if(autoIf.condition.autoCondition()){
                autoIf.preTransitionMethod.run();
                transition(autoIf.transitionNode);
                break;
            }
        }
    }

    public ArrayList <AutoIf> conditionsList;

    public LogicNode(SampleMecanumDrive dt, RobotModules robot){
        this.dt = dt;
        this.robot = robot;
    }

    private void transition(LogicNode transitionNode){
        conditionsList = transitionNode.conditionsList;
    }

}
