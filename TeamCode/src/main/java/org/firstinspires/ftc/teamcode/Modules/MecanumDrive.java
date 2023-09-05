package org.firstinspires.ftc.teamcode.Modules;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Utils.CoolMotor;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.firstinspires.ftc.teamcode.Utils.Pose;
import org.firstinspires.ftc.teamcode.Utils.CoolVector;

@Config
public class MecanumDrive implements IRobotModule {

    private final Localizer localizer;

    private final CoolMotor frontLeft, frontRight, backLeft, backRight;
    public static String frontLeftMotorName = "mfl", frontRightMotorName = "mfr",
            backLeftMotorName = "mbl", backRightMotorName = "mbr";
    public static boolean frontLeftMotorReversed = false, frontRightMotorReversed = true, backLeftMotorReversed = false, backRightMotorReversed = true;

    public static PIDCoefficients translationalPID = new PIDCoefficients(0.2,0,0),
            headingPID = new PIDCoefficients(0,0,0);
    private final PIDController tpid= new PIDController(0,0,0), hpid = new PIDController(0,0,0);

    public static double lateralMultiplier = 1.2;

    public enum RunMode{
        PID, Vector
    }

    private RunMode runMode;

    public MecanumDrive(HardwareMap hm, Localizer localizer, RunMode runMode){
        this.localizer = localizer;
        frontLeft = new CoolMotor(hm, frontLeftMotorName, CoolMotor.RunMode.RUN, frontLeftMotorReversed);
        frontRight = new CoolMotor(hm, frontRightMotorName, CoolMotor.RunMode.RUN, frontRightMotorReversed);
        backLeft = new CoolMotor(hm, backLeftMotorName, CoolMotor.RunMode.RUN, backLeftMotorReversed);
        backRight = new CoolMotor(hm, backRightMotorName, CoolMotor.RunMode.RUN, backRightMotorReversed);
        this.runMode = runMode;
    }

    public MecanumDrive(HardwareMap hm, Localizer localizer){
        this(hm, localizer, RunMode.Vector);
    }

    public CoolVector powerCoolVector = new CoolVector();
    private Pose targetPose = new Pose();
    public CoolVector targetCoolVector = new CoolVector();

    public void setTargetPose(Pose pose){
        this.targetPose = pose;
    }

    public void setTargetVector(CoolVector CoolVector){
        this.targetCoolVector = CoolVector;
    }

    public RunMode getRunMode() {
        return runMode;
    }

    public Localizer getLocalizer(){
        return localizer;
    }

    public void setRunMode(RunMode runMode){
        this.runMode = runMode;
    }

    private void updatePowerCoolVector(){
        switch (runMode){
            case Vector:
                powerCoolVector = new CoolVector(targetCoolVector.getX(), targetCoolVector.getY() * lateralMultiplier, targetCoolVector.getZ());
                break;
            case PID:
                Pose currentPose = new Pose(localizer.getPoseEstimate());

                double xDiff = targetPose.getX() - currentPose.getX();
                double yDiff = targetPose.getY() - currentPose.getY();

                double distance = Math.sqrt(xDiff * xDiff + yDiff * yDiff);

                tpid.setPID(translationalPID.p, translationalPID.i, translationalPID.d);

                double translationalPower = tpid.calculate(0, distance);

                powerCoolVector = new CoolVector(translationalPower * Math.cos(Math.atan2(yDiff, xDiff)), translationalPower * Math.sin(Math.atan2(yDiff, xDiff)), 0);

                double headingDiff = targetPose.getHeading() - currentPose.getHeading();

                while(headingDiff > PI) headingDiff -= 2.0*PI;
                while(headingDiff < -PI) headingDiff += 2.0*PI;

                hpid.setPID(headingPID.p, headingPID.i, headingPID.d);

                double headingPower = hpid.calculate(0, headingDiff);

                powerCoolVector= new CoolVector(powerCoolVector.getX(),powerCoolVector.getY(),headingPower);
                break;
        }
        powerCoolVector.scale(Math.max(1, Math.abs(powerCoolVector.getX()) + Math.abs(powerCoolVector.getY()) + Math.abs(powerCoolVector.getZ())));
    }

    private void updateMotors(){
        frontLeft.setPower(powerCoolVector.getX() - powerCoolVector.getY() - powerCoolVector.getZ());
        frontRight.setPower(powerCoolVector.getX() + powerCoolVector.getY() + powerCoolVector.getZ());
        backLeft.setPower(powerCoolVector.getX() + powerCoolVector.getY() - powerCoolVector.getZ());
        backRight.setPower(powerCoolVector.getX() - powerCoolVector.getY() + powerCoolVector.getZ());

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();
    }

    public Pose getPoseEstimate(){
        return new Pose(localizer.getPoseEstimate());
    }

    @Override
    public void atStart() {
        
    }

    @Override
    public void loop() {
        updatePowerCoolVector();
        updateMotors();
    }

    @Override
    public void emergencyStop() {
        powerCoolVector = new CoolVector();
        updateMotors();
    }
}
