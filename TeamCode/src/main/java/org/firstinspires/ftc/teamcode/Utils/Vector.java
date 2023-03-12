package org.firstinspires.ftc.teamcode.Utils;

public class Vector {
    double x,y;
    public double cx,cy;
    double angle_offset = 0;
    public double mod;

    public Vector(double x, double y){
        set_components(x,y);
        this.mod = Math.sqrt(cx*cx + cy*cy);
    }

    public void debug(){
        System.out.println("Modul din ecuatie: " + Math.sqrt(cx*cx + cy*cy));
        System.out.println("Modul: " + mod);
        System.out.println("x: " + x);
        System.out.println("y: " + y);
        System.out.println("cx: " + cx);
        System.out.println("cy: " + cy);
        System.out.println("angle_offset: " + angle_offset);
        System.out.println("===========================================================================");
    }

    public void set_angle_offset(double a){
        cx = x*Math.cos(a) + y*Math.cos(a + Math.PI/2.0);
        cy = x*Math.sin(a) + y*Math.sin(a + Math.PI/2.0);
        angle_offset = a;
    }

    public void add_angle_offset(double a){
        double new_angle_offset = angle_offset + a;
        set_angle_offset(0);
        set_angle_offset(new_angle_offset);
    }

    public void set_components(double newx, double newy){
        x = newx;
        y = newy;
        set_angle_offset(angle_offset);
    }
}
