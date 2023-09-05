package org.firstinspires.ftc.teamcode.Utils;

public class CoolVector {
    private final double x,y,z;

    public CoolVector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public CoolVector(double x, double y){
        this(x,y,0);
    }

    public CoolVector(){
        this(0,0,0);
    }

    public static CoolVector fromAngleAndMagnitude(double t, double m){
        return new CoolVector(m*Math.cos(t), m*Math.sin(t));
    }

    //t1 is angle in xy plane, t2 is angle with xy plane
    public static CoolVector fromAngleAndMagnitude(double t1, double t2, double m){
        return new CoolVector(Math.cos(t1)*Math.cos(t2)*m, Math.sin(t1)*Math.cos(t2)*m, Math.sin(t2)*m);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public CoolVector plus(CoolVector other){
        return new CoolVector(x + other.getX(), y + other.getY(), z + other.getY());
    }

    public CoolVector scale(double a){
        return new CoolVector(x * a, y * a, z * a);
    }

    @Override
    public String toString(){
        String bruh = String.valueOf(getX() + ' ' + String.valueOf(getY()) + ' ' + String.valueOf(getZ()));
        return bruh;
    }
}
