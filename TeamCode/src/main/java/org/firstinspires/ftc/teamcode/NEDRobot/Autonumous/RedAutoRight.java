package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bAutoSubsystem;
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

import javax.annotation.concurrent.GuardedBy;

@Config
@Autonomous(name = "RedAutoRight")
@Disabled
public class RedAutoRight extends LinearOpMode {

    private FtcDashboard ftcDashboard;
    private IntakeSubsystem intakeSubsystem;
    private OdometrySubsystem odometrySubsystem;

    private Dr4bAutoSubsystem dr4bAutoSubsystem;
    private SampleMecanumDrive drive;
    private VoltageSensor voltageSensor;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public double loopTime;


    private double fourbarFIRSTCONE1= 0.42;
    private double fourbarFIRSTCONE2= 0.4;
    private  double fourbarSECONDCONE=0.63;
    private double fourbarTHIRDCONE=0.63;
    private double fourbarFOURTHCONE=0.64;
    private double fourbarLASTCONE = 0.64;


    private double fourbarFIRSTPICK1 = 0.82;
    private double fourbarFIRSTPICK2 = 0.82;

    private double fourbarSECONDPICK1 = 0.84;
    private double fourbarSECONDPICK2 = 0.84;

    private double fourbarTHIRDPICK1 = 0.86;
    private double fourbarTHIRDPICK2 = 0.86;

    private double fourbarFOURTHPICK1 = 0.88;
    private double fourbarFOURTHPICK2 = 0.88;


    private double fourbarFIFTHPICK1 = 0.9;
    private double fourbarFIFTHPICK2 = 0.9;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private double imuAngle = 0;
    private Thread imuThread;


    public int HighJunctionPos = 1720;
    public int HighJunctionPosIn = 1550;
    public int HighJunctionPosOut = 1720;


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
    private TrajectorySequence Preload_drop;
    private TrajectorySequence Pick1;
    private TrajectorySequence Drop1;
    private TrajectorySequence Pick2;
    private TrajectorySequence Drop2;
    private TrajectorySequence Pick3;
    private TrajectorySequence Drop3;
    private TrajectorySequence Pick4;
    private TrajectorySequence Drop4;
    private TrajectorySequence Pick5;
    private TrajectorySequence Drop5;
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

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeSubsystem = new IntakeSubsystem(hardwareMap,true);
        odometrySubsystem = new OdometrySubsystem(hardwareMap,true);
        dr4bAutoSubsystem = new Dr4bAutoSubsystem(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //////////////////////INIT///////////////////////////////////
        odometrySubsystem.update(OdometrySubsystem.OdoState.UP);
        odometrySubsystem.update(OdometrySubsystem.OdoState.DOWN);
        intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_INTAKE);
        intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE);
        dr4bAutoSubsystem.dr4b_motor.resetEncoder();
        ////////////////////////////////////////////////////////////



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


        PhotonCore.enable();
        //trajectory

        Preload_drop = drive.trajectorySequenceBuilder(POSE_START)
                /*.splineToSplineHeading(new Pose2d(40,1.75,toRadians(0)),toRadians(0))
                .lineToSplineHeading(new Pose2d(53.3,3.8,toRadians(48.4)),
                        drive.getVelocityConstraint(45,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(40))

                 */
                .splineToSplineHeading(new Pose2d(36,1.75,toRadians(0)),toRadians(0),
                        drive.getVelocityConstraint(50,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .splineToSplineHeading(new Pose2d(53.8,3.7,toRadians(45.4)),toRadians(0),
                        drive.getVelocityConstraint(50,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))

                .build();

        ///////////////////////CYCLE 1/////////////////////////////////////////////

        Pick1 = drive.trajectorySequenceBuilder(Preload_drop.end())
                .setReversed(true)
                /*.splineTo(new Vector2d(54,-27),toRadians(270),drive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(55))
                    */
                .splineTo(new Vector2d(55,-27.5),toRadians(270),drive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(55))
                .build();

        Drop1 = drive.trajectorySequenceBuilder(Pick1.end())
                .setReversed(false)
                /*.splineTo(new Vector2d(56.3,4),toRadians(56.5),drive.getVelocityConstraint(40,toRadians(100),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(40))*/

                .splineTo(new Vector2d(56.8,3),toRadians(63),drive.getVelocityConstraint(40,toRadians(100),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(50))

                .build();

        ///////////////////////CYCLE 2/////////////////////////////////////////////

        Pick2 = drive.trajectorySequenceBuilder(Drop1.end())
                .setReversed(true)
                .splineTo(new Vector2d(53.4,-27.5),toRadians(270),drive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(55))
                .build();

        Drop2 = drive.trajectorySequenceBuilder(Pick2.end())
                .setReversed(false)
                .splineTo(new Vector2d(56.8,3),toRadians(63),drive.getVelocityConstraint(40,toRadians(100),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(50))
                .build();

        ///////////////////////CYCLE 3/////////////////////////////////////////////

        Pick3 = drive.trajectorySequenceBuilder(Drop2.end())
                .setReversed(true)
                .splineTo(new Vector2d(53,-27.5),toRadians(270),drive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(55))
                .build();

        Drop3 = drive.trajectorySequenceBuilder(Pick3.end())
                .setReversed(false)
                .splineTo(new Vector2d(56.8,3),toRadians(63),drive.getVelocityConstraint(40,toRadians(100),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(50))
                .build();

        //////////////////////CYCLE 4/////////////////////////////////

        Pick4 = drive.trajectorySequenceBuilder(Drop3.end())
                .setReversed(true)
                .splineTo(new Vector2d(53,-28),toRadians(270),drive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(55))
                .build();

        Drop4 = drive.trajectorySequenceBuilder(Pick4.end())
                .setReversed(false)
                .splineTo(new Vector2d(56,3),toRadians(63),drive.getVelocityConstraint(55,toRadians(140),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .build();

        Pick5 = drive.trajectorySequenceBuilder(Drop4.end())
                .setReversed(true)
                .splineTo(new Vector2d(53,-28),toRadians(270),drive.getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(55))
                .build();

        Drop5 = drive.trajectorySequenceBuilder(Pick5.end())
                .setReversed(false)
                .splineTo(new Vector2d(56,4),toRadians(63),drive.getVelocityConstraint(45,toRadians(140),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .build();


        ////////////////////////////PARK/////////////////////////////////////////////////////

        Park1 = drive.trajectorySequenceBuilder(Drop4.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(50,0,toRadians(90)),
                        drive.getVelocityConstraint(35,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(35))
                .lineToSplineHeading(new Pose2d(50,24.8,toRadians(90)),
                        drive.getVelocityConstraint(35,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(35))
                .build();

        Park2 = drive.trajectorySequenceBuilder(Drop4.end())
                .setReversed(false)
                .turn(toRadians(30))
                .lineToConstantHeading(new Vector2d(49,4.5),
                        drive.getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(35))
                .build();

        Park3 = drive.trajectorySequenceBuilder(Drop4.end())
                .setReversed(true)
                .splineTo(new Vector2d(48,-25),toRadians(270),
                        drive.getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(35))
                .build();


        while(!isStarted() && !isStopRequested())
        {
            drive.update();
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();


            telemetry.addLine("Running RED 5 Cycle Beleaua");


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
            telemetry.addData("Voltage", voltageSensor.getVoltage());
            telemetry.update();
        }


        waitForStart();
        camera.stopStreaming();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        //////////////////////////////PRELOAD//////////////////////////////

                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive, Preload_drop),
                                new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                                new WaitCommand(1000)
                                        .andThen(new InstantCommand(() ->dr4bAutoSubsystem.newProfile(HighJunctionPos)))
                                        .alongWith(new WaitCommand(500))
                                        .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new InstantCommand(()-> dr4bAutoSubsystem.newProfile(HighJunctionPosIn)),
                        new WaitCommand(150),//500
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(200),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosOut)),
                        new WaitCommand(200),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                                new InstantCommand(()->dr4bAutoSubsystem.newProfile(0))
                        ),
                        new WaitCommand(600),
                        //////////////////////////////PICK1//////////////////////////////

                        new ParallelCommandGroup(
                                new WaitCommand(500),
                                new FollowTrajectoryCommand(drive, Pick1),
                                new InstantCommand(()->intakeSubsystem.setFourbar( fourbarFIRSTPICK1,fourbarFIRSTPICK2)),
                                new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN))

                        ),
                        new WaitCommand(250),
                        new InstantCommand(()-> intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                        new WaitCommand(250),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,Drop1),
                                new WaitCommand(1000)
                                        .andThen(new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPos)))
                                        .alongWith(new WaitCommand(400))
                                            .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosIn)),
                        new WaitCommand(100)
                                .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)))
                                .alongWith(new WaitCommand(250))
                                    .andThen(new InstantCommand(() -> dr4bAutoSubsystem.newProfile(HighJunctionPosOut))),
                        new WaitCommand(200),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                        new WaitCommand(200),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),

                        /////////////////////////////PICK2//////////////////////////////////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive, Pick2)
                                        .alongWith(new InstantCommand(()->dr4bAutoSubsystem.newProfile(0)))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)))
                                        .andThen( new InstantCommand(()->intakeSubsystem.setFourbar( fourbarSECONDPICK1,fourbarSECONDPICK2)))

                        ),
                        new WaitCommand(200),
                        new InstantCommand(()-> intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE))
                                .alongWith(new WaitCommand(200))
                                .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,Drop2),
                                new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPos))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosIn)),
                        new WaitCommand(100),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(200),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosOut)),
                        new WaitCommand(200),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),


                        /////////////////////////PICK3//////////////////////////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive, Pick3)
                                        .alongWith(new InstantCommand(()->dr4bAutoSubsystem.newProfile(0)))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)))
                                        .andThen( new InstantCommand(()->intakeSubsystem.setFourbar( fourbarTHIRDPICK1,fourbarTHIRDPICK2)))

                        ),
                        new WaitCommand(250),
                        new InstantCommand(()-> intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE))
                                .alongWith(new WaitCommand(200))
                                .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,Drop3),
                                new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPos))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosIn)),
                        new WaitCommand(100),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(200) ,
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosOut)),
                        new WaitCommand(200),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),

                        /////////////////////////////PICK4/////////////////////////////////
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive, Pick4)
                                        .alongWith(new InstantCommand(()->dr4bAutoSubsystem.newProfile(0)))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)))
                                        .andThen( new InstantCommand(()->intakeSubsystem.setFourbar( fourbarFOURTHPICK1,fourbarFOURTHPICK2)))

                        ),
                        new WaitCommand(250),
                        new InstantCommand(()-> intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE))
                                .alongWith(new WaitCommand(200))
                                .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,Drop4),
                                new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPos))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosIn)),
                        new WaitCommand(100),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(200) ,
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosOut)),
                        new WaitCommand(200),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),

                        //////////////////////PICK5/////////////////


                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive, Pick5)
                                        .alongWith(new InstantCommand(()->dr4bAutoSubsystem.newProfile(0)))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)))
                                        .andThen( new InstantCommand(()->intakeSubsystem.setFourbar( fourbarFIFTHPICK1,fourbarFIFTHPICK2)))

                        ),
                        new WaitCommand(250),
                        new InstantCommand(()-> intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE))
                                .alongWith(new WaitCommand(200))
                                .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,Drop5),
                                new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPos))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosIn)),
                        new WaitCommand(100),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)),
                        new WaitCommand(200) ,
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosOut)),
                        new WaitCommand(200),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(0))


                        //////////////////////////////PARK//////////////////////////////

                      /*  new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,position == 1? Park1 : position == 2?Park2 : Park3),
                                new InstantCommand(()-> intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE))
                                        .andThen(new InstantCommand(()-> intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
                                                .andThen(new InstantCommand(()-> dr4bAutoSubsystem.newProfile(0)))
                                                .andThen( new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN))))

                        )

                       */


                        //new InstantCommand(this::requestOpModeStop)
                )

        );
        while(opModeIsActive())
        {
            dr4bAutoSubsystem.read();
            dr4bAutoSubsystem.loop();
            CommandScheduler.getInstance().run();
            dr4bAutoSubsystem.write();
            drive.update();
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
    /*public void update(int pos) {
        dr.setTargetPosition(pos);
        dr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr.setPower(1);
    }*/
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