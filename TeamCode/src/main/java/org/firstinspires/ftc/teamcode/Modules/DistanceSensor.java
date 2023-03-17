package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.random.AbstractRandomGenerator;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;

import java.lang.reflect.Array;
import java.util.Arrays;

@Config
public class DistanceSensor implements IRobotModule {

    public double value;
    public static double valuesForMedian = 4;
    private double[] lastValues = {1000};

    Rev2mDistanceSensor sensor;

    public DistanceSensor(HardwareMap hm){
        sensor = hm.get(Rev2mDistanceSensor.class, "guide");
    }

    @Override
    public void atStart() {

    }

    private void addValue(double x){
        if(lastValues.length < valuesForMedian) {
            double[] aux = Arrays.copyOf(lastValues, lastValues.length+1);
            aux[lastValues.length] = x;
            lastValues = aux;
            return;
        }
        double[] aux = Arrays.copyOf(lastValues, lastValues.length);
        for(int i =0;i<lastValues.length -1;i++) aux[i] = aux[i+1];
        aux[lastValues.length - 1] = x;
        lastValues = aux;
    }

    private double getMedian(double[] a){
        double[] aux = Arrays.copyOf(a, a.length);
        Arrays.sort(aux);
        return aux[aux.length/2];
    }

    @Override
    public void loop() {
        addValue(sensor.getDistance(DistanceUnit.MM));
        value = getMedian(lastValues);
    }
}
