package org.firstinspires.ftc.teamcode.NEDRobot.Vision;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Vision {


    public static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    public double fx = 578.272;
    public double fy = 578.272;
    public double cx = 402.145;
    public double cy = 221.506;

    public FtcDashboard ftcDashboard;

    // UNITS ARE METERS
    public double tagsize = 0.166;

    public int position = 1;


    public int LEFT = 1 ; // Tag ID 18 from the 36h11 family
    public int MIDDLE = 2;
    public int RIGHT= 3;
    public OpenCvCamera camera;
    public AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public AprilTagDetection tagOfInterest = null;
    public int cameraMonitorViewId;

    public Vision(HardwareMap hardwareMap)
    {
        this.cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        this.aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        this.camera.setPipeline(aprilTagDetectionPipeline);
        this.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
    public void update()
    {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
        }

        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            position = 1;
        }else if(tagOfInterest.id == MIDDLE){
            position =2;
        }else{
            position = 3;
        }

    }
    public int getPosition()
    {
        return position;
    }
}
