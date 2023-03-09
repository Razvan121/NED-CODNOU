package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

import static java.lang.Math.toRadians;

import androidx.annotation.GuardedBy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Disabled
@Autonomous(name = "AutoLeft", group="RED")
public class  RedAutoLeft extends LinearOpMode {

    private FtcDashboard ftcDashboard;
    private IntakeSubsystem intakeSubsystem;
    private OdometrySubsystem odometrySubsystem;
    private SampleMecanumDrive drive;
    DcMotorEx dr;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public double loopTime;


    private double fourbarFIRSTCONE= 0.63;
    private  double fourbarSECONDCONE=0.63;
    private double fourbarTHIRDCONE=0.63;
    private double fourbarFOURTHCONE=0.64;
    private double fourbarLASTCONE = 0.64;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private double imuAngle = 0;
    private Thread imuThread;


    public int HighJunctionPos = 1500;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT = 1 ; // Tag ID 18 from the 36h11 family
    int MIDDLE = 2;
    int RIGHT= 3;

    int position = 1;

    AprilTagDetection tagOfInterest = null;

    //TrajectorySequence
    private TrajectorySequence Beleaua;
    private TrajectorySequence Spate;
    private TrajectorySequence Pick1;
    private TrajectorySequence Drop1;
    private TrajectorySequence Pick2;
    private TrajectorySequence Drop2;
    private TrajectorySequence Pick3;
    private TrajectorySequence Drop3;
    private Trajectory Pick4;
    private Trajectory Drop4;
    private Trajectory Pick5;
    private Trajectory Drop5;
    private TrajectorySequence Park1;
    private TrajectorySequence Park2;
    private TrajectorySequence Park3;

    //Pose
    public static Pose2d POSE_START = new Pose2d(0,0,toRadians(0));

    public static Pose2d[] CYCLE_DROP = new Pose2d[]{
            new Pose2d(55,2.4,toRadians(14)),//preload
            new Pose2d(49,0,toRadians(0)),//pick1
            new Pose2d(0,0,toRadians(0)),//pick2
            new Pose2d(0,0,toRadians(0)),//pick3
            new Pose2d(0,0,toRadians(0)),//pick4
            new Pose2d(0,0,toRadians(0)),//pick5

    };

    public static Pose2d[] CYCLE_PICK = new Pose2d[]{
            new Pose2d(0,0,toRadians(0)),//drop1
            new Pose2d(0,0,toRadians(0)),//drop2
            new Pose2d(0,0,toRadians(0)),//drop3
            new Pose2d(0,0,toRadians(0)),//drop4
            new Pose2d(0,0,toRadians(0)),//drop5
    };

    public static Vector2d PARK = new Vector2d(52,-23);//-23


    @Override
    public void runOpMode() throws RuntimeException{
        CommandScheduler.getInstance().reset();

        dr = hardwareMap.get(DcMotorEx.class,"dr4b");

        intakeSubsystem = new IntakeSubsystem(hardwareMap,true);
        odometrySubsystem = new OdometrySubsystem(hardwareMap,true);
        drive = new SampleMecanumDrive(hardwareMap);

        intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_INTAKE);

        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
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
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();

        startIMUThread(this);

        //localizer
        drive.getLocalizer().setPoseEstimate(POSE_START);

        //trajectory
        Beleaua = drive.trajectorySequenceBuilder(POSE_START)
                .lineToConstantHeading(new Vector2d(70,0.5),
                        drive.getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(60))
                .build();


        Spate = drive.trajectorySequenceBuilder(Beleaua.end())
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(50,0.5),
                        drive.getVelocityConstraint(45,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .build();


        ////////////////////////////PARK/////////////////////////////////////////////////////

        Park1 = drive.trajectorySequenceBuilder(Spate.end())
                .setReversed(false)
                .strafeTo(new Vector2d(49,25))
                .build();

        Park2 = drive.trajectorySequenceBuilder(Spate.end())
                .setReversed(false)
                .strafeTo(new Vector2d(52,0))
                .build();

        Park3 = drive.trajectorySequenceBuilder(Spate.end())
                .setReversed(false)
                .strafeTo(new Vector2d(51,-23))
                .build();


        while(!isStarted())
        {
            drive.update();
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

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            if(tagOfInterest == null || tagOfInterest.id == LEFT){
                position = 1;
            }else if(tagOfInterest.id == MIDDLE){
                position =2;
            }else{
                position = 3;
            }


            telemetry.addLine("start");
            telemetry.update();
        }


        waitForStart();
        // camera.stopStreaming();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                        new FollowTrajectoryCommand(drive,Beleaua),
                        new FollowTrajectoryCommand(drive,Spate),
                        new FollowTrajectoryCommand(drive,position == 1? Park1 : position == 2?Park2 : Park3)
                        //new InstantCommand(this::requestOpModeStop)
                )

        );
        //robot.reset();

        while(opModeIsActive())
        {
            CommandScheduler.getInstance().run();
            drive.update();
          /*  telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            double loop = System.nanoTime();
            telemetry.addData("hz", 1000000000/(loop-loopTime));
            loopTime= loop;
            telemetry.update();
           */
        }
    }


    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public void update(int pos) {
        dr.setTargetPosition(pos);
        dr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr.setPower(0.8);
    }
    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested() && opMode.opModeIsActive()) {
                synchronized (imuLock) {
                    imuAngle = -imu.getAngularOrientation().firstAngle;
                }
            }
        });
        imuThread.start();
    }

}