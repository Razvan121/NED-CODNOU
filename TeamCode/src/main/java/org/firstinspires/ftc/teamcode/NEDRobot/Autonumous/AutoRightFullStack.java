package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

import static org.firstinspires.ftc.teamcode.NEDRobot.Autonumous.AutoRightBeleaua.CYCLE_DROP;
import static org.firstinspires.ftc.teamcode.NEDRobot.Autonumous.AutoRightBeleaua.CYCLE_PICK;
import static org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive.getVelocityConstraint;
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


@Autonomous(name= "1+5 HIGH-RIGHT")
@Config

public class AutoRightFullStack extends LinearOpMode {

    private FtcDashboard dashboard;

    private IntakeSubsystem intake;
    private Dr4bAutoSubsystem lift;
    private OdometrySubsystem odometry;
    //private SampleMecanumDrive drive;

  //  private Vision vision;

    private BaseRobotAuto robot;

    private VoltageSensor voltageSensor;

    private int position;

    private final double fourbarFIRSTPICK1 = 0.70;
    private final double fourbarFIRSTPICK2 = 0.70;

    private final double fourbarSECONDPICK1 = 0.73;
    private final double fourbarSECONDPICK2 = 0.73;

    private final double fourbarTHIRDPICK1 = 0.75;
    private final double fourbarTHIRDPICK2 = 0.75;

    private final double fourbarFOURTHPICK1 = 0.77;
    private final double fourbarFOURTHPICK2 = 0.77;

    private final double fourbarFIFTHPICK1 = 0.77;
    private final double fourbarFIFTHPICK2 = 0.77;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;

    private double imuAngle = 0;

    private Thread imuThread;

    public int HighJunctionPos = 16300;
    public int HighJunctionPosIn = 1420;
    public int HighJunctionPosOut = 1590;

    private TrajectorySequence preload;

    private TrajectorySequence pick1;

    private TrajectorySequence drop1;

    private TrajectorySequence pick2;

    private TrajectorySequence drop2;

    private TrajectorySequence pick3;

    private TrajectorySequence drop3;

    private TrajectorySequence pick4;

    private TrajectorySequence drop4;

    private TrajectorySequence pick5;

    private TrajectorySequence drop5;

    private TrajectorySequence park1,park2,park3;




    public static Pose2d POSE_START = new Pose2d(0,0, toRadians(0));




    @Override
    public void runOpMode() throws InterruptedException {

        CommandScheduler.getInstance().reset();

        telemetry =  new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new IntakeSubsystem(hardwareMap,true);
        odometry = new OdometrySubsystem(hardwareMap,true);
        lift = new Dr4bAutoSubsystem(hardwareMap);
       // drive = new SampleMecanumDrive(hardwareMap);
        //vision = new Vision(hardwareMap);

        robot = new BaseRobotAuto(hardwareMap);


        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        ////////////////////INIT////////////////

        odometry.update(OdometrySubsystem.OdoState.UP);
        odometry.update(OdometrySubsystem.OdoState.DOWN);
        intake.update((IntakeSubsystem.FourbarState.TRANSITION_INTAKE));
        intake.update(IntakeSubsystem.ClawState.CLOSE);
        lift.dr4b_motor.resetEncoder();


        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }


        FtcDashboard.getInstance().startCameraStream(robot.vision.camera,30);

        telemetry.setMsTransmissionInterval(50);

        dashboard = FtcDashboard.getInstance();

        startIMUThread(this);

        //localizer
        robot.drivetrain.getLocalizer().setPoseEstimate(POSE_START);


        PhotonCore.enable();


        preload = robot.drivetrain.trajectorySequenceBuilder(POSE_START)
                .splineToLinearHeading(new Pose2d(53.8,3.7,toRadians(49)),toRadians(13),
                getVelocityConstraint(60,toRadians(120), DriveConstants.TRACK_WIDTH),
                getAccelerationConstraint(60))
                .build();


        pick1 =  robot.drivetrain.trajectorySequenceBuilder(preload.end())
                .setReversed(true)
                .splineTo(new Vector2d(54,-25),toRadians(270),getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))//3 61)
                .build();

        drop1 =  robot.drivetrain.trajectorySequenceBuilder(preload.end())
                .splineTo(new Vector2d(53,3.4),toRadians(55),getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))//3 61)
                .build();
        pick2 = robot.drivetrain.trajectorySequenceBuilder(drop1.end())
                .setReversed(true)
                .splineTo(CYCLE_PICK[1],toRadians(270), getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))
                .build();
        drop2 = robot.drivetrain.trajectorySequenceBuilder(pick2.end())
                .setReversed(false)
                .splineTo(CYCLE_DROP[1],toRadians(54.3), getVelocityConstraint(65,toRadians(140),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))//3.3
                .build();

        ///////////////////////CYCLE 3/////////////////////////////////////////////

        pick3 = robot.drivetrain.trajectorySequenceBuilder(drop2.end())
                .setReversed(true)
                .splineTo(CYCLE_PICK[2],toRadians(270), getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))
                .build();

        drop3 = robot.drivetrain.trajectorySequenceBuilder(pick3.end())
                .setReversed(false)
                .splineTo(CYCLE_DROP[2],toRadians(53), getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))//4.2 50
                .build();

        //////////////////////CYCLE 4/////////////////////////////////

        pick4 = robot.drivetrain.trajectorySequenceBuilder(drop3.end())
                .setReversed(true)
                .splineTo(CYCLE_PICK[3],toRadians(270), getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))
                .build();

        drop4 = robot.drivetrain.trajectorySequenceBuilder(pick4.end())
                .setReversed(false)
                .splineTo(CYCLE_DROP[3],toRadians(60), getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))//4.2 50
                .build();

        //////////////////////CYCLE 5/////////////////////////////////


        pick5 = robot.drivetrain.trajectorySequenceBuilder(drop4.end())
                .setReversed(true)
                .splineTo(new Vector2d(53,-25),toRadians(270), getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))
                .build();

        drop5 = robot.drivetrain.trajectorySequenceBuilder(pick5.end())
                .setReversed(false)
                .splineTo(new Vector2d(56,4),toRadians(73), getVelocityConstraint(45,toRadians(140),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(45))
                .build();


        ////////////////////////////PARK/////////////////////////////////////////////////////

        park1 = robot.drivetrain.trajectorySequenceBuilder(drop4.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(50,0,toRadians(90)),
                        getVelocityConstraint(55,toRadians(160),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))
                .lineToSplineHeading(new Pose2d(50,25,toRadians(90)),
                        getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(70))//55 120
                .build();

        park2 = robot.drivetrain.trajectorySequenceBuilder(drop4.end())
                .setReversed(false)
                .turn(toRadians(30))
                .lineToConstantHeading(new Vector2d(47.7,4.5),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(70))
                .build();

        park3 = robot.drivetrain.trajectorySequenceBuilder(drop4.end())
                .setReversed(true)
                .splineTo(new Vector2d(48,-25),toRadians(270),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(70))
                .build();

        while( !isStarted() && !isStopRequested())
        {
            robot.drivetrain.update();
            robot.vision.update();
            position = robot.vision.getPosition();


            telemetry.addLine("start");
            telemetry.addLine("Running 5 Cycle HIGH");
            telemetry.addData("position",robot.vision.getPosition());
            telemetry.addData("Voltage", voltageSensor.getVoltage());
            telemetry.update();
        }

        waitForStart();
        robot.vision.camera.stopStreaming();

        startIMUThread(this);


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(robot.drivetrain, preload),
                                new InstantCommand(()->intake.update(IntakeSubsystem.ClawState.CLOSE)),
                                new InstantCommand(()->intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                                new WaitCommand(500)
                                        .andThen(new InstantCommand(()-> lift.newProfile(HighJunctionPos)))
                        ),
                        new WaitCommand(350),
                        new InstantCommand(()->intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                        new InstantCommand(()->intake.update(IntakeSubsystem.ClawState.OPEN)),

                        new ParallelCommandGroup(
                                new WaitCommand(1000)
                                        .andThen(new FollowTrajectoryCommand(robot.drivetrain, pick1)),
                                new InstantCommand(()-> intake.update(IntakeSubsystem.ClawState.CLOSE))
                                        .andThen(new WaitCommand(300))
                                        .andThen(new InstantCommand(()-> intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
                                                .andThen(new InstantCommand(()-> lift.newProfile(0)))
                                                .andThen(new WaitCommand(100))
                                                .andThen( new InstantCommand(()->intake.update(IntakeSubsystem.ClawState.OPEN))))
                                        .andThen(new WaitCommand(700))
                                        .andThen(new InstantCommand(()-> intake.setFourbar(fourbarFIRSTPICK1,fourbarFIRSTPICK2)))
                        )



                )
        );



        while(opModeIsActive())
        {
            CommandScheduler.getInstance().run();


            lift.read();
            lift.loop();
            lift.write();
            robot.drivetrain.update();


            telemetry.addData("ticks",lift.getDr4bPosition());
            telemetry.update();
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
