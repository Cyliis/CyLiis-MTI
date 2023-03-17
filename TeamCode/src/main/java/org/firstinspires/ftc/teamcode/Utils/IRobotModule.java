package org.firstinspires.ftc.teamcode.Utils;

public interface IRobotModule {
    public static boolean ENABLE_MODULE = false;
    void atStart();
    void loop();
    default void emergencyStop() {};
    default boolean isActive(){
        return ENABLE_MODULE;
    }
}