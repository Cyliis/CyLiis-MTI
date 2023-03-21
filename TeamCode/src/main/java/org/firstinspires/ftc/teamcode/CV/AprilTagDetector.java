package org.firstinspires.ftc.teamcode.CV;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utils.IRobotModule;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;


@Config
public class AprilTagDetector implements IRobotModule {
    public static boolean DISPLAY_ON_DASHBOARD = false;
    public static boolean DISPLAY_ON_SCREEN = true;
    public static boolean ENABLE_MODULE = true;

    public enum DetectionResult {
        UNKNOWN,
        Left,
        Middle,
        Right,
    }

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    static final double fx = 578.272;
    static final double fy = 578.272;
    static final double cx = 402.145;
    static final double cy = 221.506;

    // UNITS ARE METERS
    static final double tagsize = 0.116;

    // Tag ID 1,2,3 from the 36h11 family
    static final int LEFT = 1;
    static final int MIDDLE = 2;
    static final int RIGHT = 3;

    Telemetry tele;

    AprilTagDetectionPipeline pipeline;
    OpenCvCamera camera;

    AprilTagDetection tagOfInterest = null;

    public AprilTagDetector(HardwareMap hm, Telemetry tele) {
        this.tele = tele;

        // initialise camera
        int cameraMonitorViewId = hm.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hm.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hm.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // create pipeline
        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(pipeline);

        //
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { }
        });

        // dashboard streaming
        if (DISPLAY_ON_DASHBOARD && FtcDashboard.getInstance() != null)
            FtcDashboard.getInstance().startCameraStream(camera, 10);

    }

    public void closeCamera(){
        camera.closeCameraDevice();
    }

    @Override
    public void atStart() {

    }

    @Override
    public void loop() {
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                {
                    tagOfInterest = tag;
                    break;
                }
            }
        }
        if (tagOfInterest != null)
            tagToTelemetry(tagOfInterest);
    }

    @Override
    public void emergencyStop() {
        camera.pauseViewport();
        if (FtcDashboard.getInstance() != null)
            FtcDashboard.getInstance().stopCameraStream();
        camera.closeCameraDeviceAsync(() -> { });
    }

    public DetectionResult getResult(){
        if (tagOfInterest == null)
            return DetectionResult.UNKNOWN;
        switch (tagOfInterest.id) {
            case LEFT:
                return DetectionResult.Left;

            case MIDDLE:
                return DetectionResult.Middle;
            case RIGHT:
                return DetectionResult.Right;
        }
        return DetectionResult.UNKNOWN;
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        tele.addLine(String.format("\nDetected tag ID=%d", detection.id));
        tele.addLine(String.format("Translation X: %.2f m", detection.pose.x));
        tele.addLine(String.format("Translation Y: %.2f m", detection.pose.y));
        tele.addLine(String.format("Translation Z: %.2f m", detection.pose.z));
        tele.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        tele.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        tele.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}