package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

import static java.lang.Math.toRadians;

import androidx.annotation.GuardedBy;

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
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobotAuto;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bAutoSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Vision.Vision;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;
import java.util.ArrayList;

@Config
@Disabled
@Autonomous(name = "AutoRightNational")
public class AutoRightNational extends LinearOpMode {

    private FtcDashboard ftcDashboard;

    //private BaseRobotAuto robot;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private double imuAngle = 0;
    private Thread imuThread;
    
    
    private IntakeSubsystem intakeSubsystem;
    private Dr4bAutoSubsystem dr4bAutoSubsystem;
    private OdometrySubsystem odometrySubsystem;
    private SampleMecanumDrive drive;
    private VoltageSensor voltageSensor;
    private double fourbarFIRSTPICK1 = 0.15;
    private double fourbarFIRSTPICK2 = 0.12;

    private double fourbarSECONDPICK1 = 0.11;
    private double fourbarSECONDPICK2 = 0.08;

    private double fourbarTHIRDPICK1 = 0.085;
    private double fourbarTHIRDPICK2 = 0.055;

    private double fourbarFOURTHPICK1 = 0.07;
    private double fourbarFOURTHPICK2 = 0.03;


    private double fourbarFIFTHPICK1 = 0.02;
    private double fourbarFIFTHPICK2 = 0.0;

    public int HighJunctionPos = 1685;
    public int HighJunctionPosIn = 1350;
    public int HighJunctionPosOut = 1640;
    int position = 1;

    //TrajectorySequence
    private TrajectorySequence Preload_drop;
    private TrajectorySequence Pick1;

    public static double xP1=54,yP1=-27,angleP1=290;
    public static double xD1=55.8,yD1=0,angleD1=55;
   // public static double xP2=50,yP2=-25.5,angleP2=0;
    //public static double xD2=50,yD2=-25.5,angleD2=0;
    //public static double xP3=50,yP3=-25.5,angleP3=0;
    //public static double xD3=50,yD3=-25.5,angleD3=0;

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

    private Vision vision;


    //Pose
    public static Pose2d POSE_START = new Pose2d(0,0,toRadians(0));

    @Override
    public void runOpMode() throws RuntimeException{

        CommandScheduler.getInstance().reset();
        vision = new Vision(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //robot = new BaseRobotAuto(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap, true);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, true);
        dr4bAutoSubsystem = new Dr4bAutoSubsystem(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //////////////////////INIT///////////////////////////////////
        //robot.init();
        intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE);
        odometrySubsystem.update(OdometrySubsystem.OdoState.UP);
        odometrySubsystem.update(OdometrySubsystem.OdoState.DOWN);
        intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_INTAKE);
        dr4bAutoSubsystem.dr4b_motor.resetEncoder();
        ////////////////////////////////////////////////////////////

        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }
        
        FtcDashboard.getInstance().startCameraStream(vision.camera, 30);

        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();


        //localizer
        drive.getLocalizer().setPoseEstimate(POSE_START);


        PhotonCore.enable();


        //trajectory

        Preload_drop = drive.trajectorySequenceBuilder(POSE_START)
                .splineToSplineHeading(new Pose2d(36,1.75,toRadians(0)),toRadians(0),
                        drive.getVelocityConstraint(45,toRadians(140), DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .splineToSplineHeading(new Pose2d(55,3,toRadians(61)),toRadians(0),
                        drive.getVelocityConstraint(45,toRadians(140),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .build();


        ///////////////////////CYCLE 1/////////////////////////////////////////////

        Pick1 = drive.trajectorySequenceBuilder(Preload_drop.end())
                .setReversed(true)
                .splineTo(new Vector2d(xP1,yP1),toRadians(angleP1),drive.getVelocityConstraint(30,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(30))
                .build();

        Drop1 = drive.trajectorySequenceBuilder(Pick1.end())
                .setReversed(false)
                .splineTo(new Vector2d(xD1,yD1),toRadians(angleD1),drive.getVelocityConstraint(45,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))

                .build();

        ///////////////////////CYCLE 2/////////////////////////////////////////////

        Pick2 = drive.trajectorySequenceBuilder(Drop1.end())
                .setReversed(true)
                .splineTo(new Vector2d(50,-25.5),toRadians(270),drive.getVelocityConstraint(45,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .build();

        Drop2 = drive.trajectorySequenceBuilder(Pick2.end())
                .setReversed(false)
                .splineTo(new Vector2d(55.8,0),toRadians(55),drive.getVelocityConstraint(45,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .build();

        ///////////////////////CYCLE 3/////////////////////////////////////////////

        Pick3 = drive.trajectorySequenceBuilder(Drop2.end())
                .setReversed(true)
                .splineTo(new Vector2d(50,-25.7),toRadians(270),drive.getVelocityConstraint(45,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .build();

        Drop3 = drive.trajectorySequenceBuilder(Pick3.end())
                .setReversed(false)
                .splineTo(new Vector2d(55.8,0),toRadians(55),drive.getVelocityConstraint(45,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(45))
                .build();

        //////////////////////CYCLE 4/////////////////////////////////

        Pick4 = drive.trajectorySequenceBuilder(Drop3.end())
                .setReversed(true)
                .splineTo(new Vector2d(50,-25.5),toRadians(270),drive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(50))
                .build();

        Drop4 = drive.trajectorySequenceBuilder(Pick4.end())
                .setReversed(false)
                .splineTo(new Vector2d(54,0),toRadians(57),drive.getVelocityConstraint(50,toRadians(140),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(50))
                .build();

        Pick5 = drive.trajectorySequenceBuilder(Drop4.end())
                .setReversed(true)
                .splineTo(new Vector2d(50,-25.5),toRadians(270),drive.getVelocityConstraint(50,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(50))
                .build();

        Drop5 = drive.trajectorySequenceBuilder(Pick5.end())
                .setReversed(false)
                .splineTo(new Vector2d(54,0),toRadians(57),drive.getVelocityConstraint(50,toRadians(140),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(50 ))
                .build();


        ////////////////////////////PARK/////////////////////////////////////////////////////

        Park1 = drive.trajectorySequenceBuilder(Drop4.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(50,0,toRadians(90)),
                        drive.getVelocityConstraint(65,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(65))
                .lineToSplineHeading(new Pose2d(50,24.8,toRadians(90)),
                        drive.getVelocityConstraint(65,toRadians(120),DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(65))
                .build();

        Park2 = drive.trajectorySequenceBuilder(Drop4.end())
                .setReversed(false)
                .turn(toRadians(30))
                .lineToConstantHeading(new Vector2d(49,4.5),
                        drive.getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(65))
                .build();

        Park3 = drive.trajectorySequenceBuilder(Drop4.end())
                .setReversed(true)
                .splineTo(new Vector2d(48,-25),toRadians(270),
                        drive.getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        drive.getAccelerationConstraint(65))
                .build();


        while(!isStarted() && !isStopRequested())
        {
            drive.update();
            vision.update();
            position=vision.getPosition();
            telemetry.addLine("Running RED 5 Cycle Beleaua");
            telemetry.addData("ID",position);
            telemetry.addData("Voltage", voltageSensor.getVoltage());
            telemetry.update();
        }

        waitForStart();
        vision.camera.stopStreaming();
        startIMUThread(this);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowTrajectoryCommand(drive, Preload_drop),
                        new FollowTrajectoryCommand(drive, Pick1),
                        new FollowTrajectoryCommand(drive, Drop1),
                        /*new FollowTrajectoryCommand(drive, Pick2),
                        new FollowTrajectoryCommand(drive, Drop2),
                        new FollowTrajectoryCommand(drive, Pick3),
                        new FollowTrajectoryCommand(drive, Drop3),
                        new FollowTrajectoryCommand(drive, Pick4),
                        new FollowTrajectoryCommand(drive, Drop4),
                        new FollowTrajectoryCommand(drive, Pick5),
                        new FollowTrajectoryCommand(drive, Drop5),
                        new FollowTrajectoryCommand(drive,position == 1? Park1 : position == 2?Park2 : Park3),

                         */

                        //////////////////////////////PRELOAD//////////////////////////////
                     /*   new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive, Preload_drop)
                               /* new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                                new WaitCommand(500).andThen(new InstantCommand(() ->dr4bAutoSubsystem.newProfile(HighJunctionPos)))
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
                        //////////////////////////////PICK1//////////////////////////////

                        new ParallelCommandGroup(
                                new InstantCommand(()->intakeSubsystem.setFourbar( fourbarFIRSTPICK1,fourbarFIRSTPICK2))
                                        .alongWith( new FollowTrajectoryCommand(drive, Pick1))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)))

                        ),
                        new WaitCommand(250),
                        new InstantCommand(()-> intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                        new WaitCommand(200),
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,Drop1),
                                new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPos))
                                        .alongWith(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosIn)),
                        new WaitCommand(100)
                                .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN)))
                                .alongWith(new WaitCommand(200))
                                .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)))
                                .alongWith(new InstantCommand(()->dr4bAutoSubsystem.newProfile(HighJunctionPosOut))),
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
                        new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE))
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
                        new InstantCommand(()->dr4bAutoSubsystem.newProfile(0)),


                        //////////////////////////////PARK//////////////////////////////

                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(drive,position == 1? Park1 : position == 2?Park2 : Park3),
                                new InstantCommand(()-> intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE))
                                        .andThen(new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
                                                .andThen(new InstantCommand(()-> dr4bAutoSubsystem.newProfile(0)))
                                                .andThen( new InstantCommand(()->intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN))))

                        ),
                        */




                        new InstantCommand(this::requestOpModeStop)
                )

        );
        while(opModeIsActive())
        {
            dr4bAutoSubsystem.read();
            dr4bAutoSubsystem.loop();
            drive.update();
            drive.updatePoseEstimate();
            
            CommandScheduler.getInstance().run();
            dr4bAutoSubsystem.write();
        }
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