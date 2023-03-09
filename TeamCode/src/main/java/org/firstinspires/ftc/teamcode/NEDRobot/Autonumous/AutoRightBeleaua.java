package org.firstinspires.ftc.teamcode.NEDRobot.Autonumous;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.autoCommands.AutoDropCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.autoCommands.AutoPickCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobotAuto;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "1+4 HIGH-RIGHT")
public class AutoRightBeleaua extends LinearOpMode {

    private FtcDashboard ftcDashboard;
  /*  private IntakeSubsystem intakeSubsystem;
    private OdometrySubsystem odometrySubsystem;

    private Dr4bAutoSubsystem dr4bAutoSubsystem;
    private SampleMecanumDrive drive;

   */
    public BaseRobotAuto robot;
    private VoltageSensor voltageSensor;

    private int position;

    private final double fourbarFIRSTPICK1 = 0.66;
    private final double fourbarFIRSTPICK2 = 0.66;

    private final double fourbarSECONDPICK1 = 0.69;
    private final double fourbarSECONDPICK2 = 0.69;

    private final double fourbarTHIRDPICK1 = 0.71;
    private final double fourbarTHIRDPICK2 = 0.71;

    private final double fourbarFOURTHPICK1 = 0.73;
    private final double fourbarFOURTHPICK2 = 0.73;


    private final double fourbarFIFTHPICK1 = 0.77;
    private final double fourbarFIFTHPICK2 = 0.77;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private double imuAngle = 0;
    private Thread imuThread;


    public int HighJunctionPos = 1590;
    public int HighJunctionPosIn = 1420;
    public int HighJunctionPosOut = 1590;


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


    public static Vector2d[] CYCLE_DROP = new Vector2d[]{
            new Vector2d(55,3.4),//drop1
            new Vector2d(55,4.2),//drop2
            new Vector2d(55.5,4.2),//drop3
            new Vector2d(55.5,4.6),//drop4
            new Vector2d(0,0),//drop5

    };
    public static Vector2d[] CYCLE_PICK = new Vector2d[]{
            new Vector2d(54,-25),//pick1
            new Vector2d(53.4,-25),//pick2
            new Vector2d(52.5,-25),//pick3
            new Vector2d(52,-24.8),//pick4
            new Vector2d(0,0),//drop5

    };

    //Pose
    public static Pose2d POSE_START = new Pose2d(0,0,toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException{
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new BaseRobotAuto(hardwareMap);


       /* intakeSubsystem = new IntakeSubsystem(hardwareMap,true);
        odometrySubsystem = new OdometrySubsystem(hardwareMap,true);
        dr4bAutoSubsystem = new Dr4bAutoSubsystem(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        vision = new Vision(hardwareMap);

        */

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //////////////////////INIT///////////////////////////////////
        robot.odometry.update(OdometrySubsystem.OdoState.UP);
        robot.odometry.update(OdometrySubsystem.OdoState.DOWN);
        robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_INTAKE);
        robot.intake.update(IntakeSubsystem.ClawState.CLOSE);
        robot.lift.dr4b_motor.resetEncoder();
        ////////////////////////////////////////////////////////////


        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

        FtcDashboard.getInstance().startCameraStream(robot.vision.camera,30);

        telemetry.setMsTransmissionInterval(50);

        ftcDashboard = FtcDashboard.getInstance();

        startIMUThread(this);

        //localizer
        robot.drivetrain.getLocalizer().setPoseEstimate(POSE_START);


        PhotonCore.enable();
        //trajectory

        Preload_drop = robot.drivetrain.trajectorySequenceBuilder(POSE_START)
                .splineToSplineHeading(new Pose2d(36,1.75,toRadians(0)),toRadians(0),
                        getVelocityConstraint(55,toRadians(120),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(45))
                .splineToSplineHeading(new Pose2d(53.8,3.7,toRadians(49)),toRadians(0),
                        getVelocityConstraint(50,toRadians(120),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(45))

                .build();


        ///////////////////////CYCLE 1/////////////////////////////////////////////

        Pick1 = robot.drivetrain.trajectorySequenceBuilder(Preload_drop.end())
                .setReversed(true)
                .splineTo(CYCLE_PICK[0],toRadians(270), getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))
                .build();

        Drop1 = robot.drivetrain.trajectorySequenceBuilder(Pick1.end())
                .setReversed(false)
                .splineTo(CYCLE_DROP[0],toRadians(55), getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))//3 61

                .build();

        ///////////////////////CYCLE 2/////////////////////////////////////////////

        Pick2 = robot.drivetrain.trajectorySequenceBuilder(Drop1.end())
                .setReversed(true)
                .splineTo(CYCLE_PICK[1],toRadians(270), getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))
                .build();

        Drop2 = robot.drivetrain.trajectorySequenceBuilder(Pick2.end())
                .setReversed(false)
                .splineTo(CYCLE_DROP[1],toRadians(54.3), getVelocityConstraint(45,toRadians(140),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(45))//3.3
                .build();

        ///////////////////////CYCLE 3/////////////////////////////////////////////

        Pick3 = robot.drivetrain.trajectorySequenceBuilder(Drop2.end())
                .setReversed(true)
                .splineTo(CYCLE_PICK[2],toRadians(270), getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))
                .build();

        Drop3 = robot.drivetrain.trajectorySequenceBuilder(Pick3.end())
                .setReversed(false)
                .splineTo(CYCLE_DROP[2],toRadians(53), getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))//4.2 50
                .build();

        //////////////////////CYCLE 4/////////////////////////////////

        Pick4 = robot.drivetrain.trajectorySequenceBuilder(Drop3.end())
                .setReversed(true)
                .splineTo(CYCLE_PICK[3],toRadians(270), getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))
                .build();

        Drop4 = robot.drivetrain.trajectorySequenceBuilder(Pick4.end())
                .setReversed(false)
                .splineTo(CYCLE_DROP[3],toRadians(60), getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))//4.2 50
                .build();

        //////////////////////CYCLE 5/////////////////////////////////


        Pick5 = robot.drivetrain.trajectorySequenceBuilder(Drop4.end())
                .setReversed(true)
                .splineTo(new Vector2d(53,-25),toRadians(270), getVelocityConstraint(55,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))
                .build();

        Drop5 = robot.drivetrain.trajectorySequenceBuilder(Pick5.end())
                .setReversed(false)
                .splineTo(new Vector2d(56,4),toRadians(73), getVelocityConstraint(45,toRadians(140),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(45))
                .build();


        ////////////////////////////PARK/////////////////////////////////////////////////////

        Park1 = robot.drivetrain.trajectorySequenceBuilder(Drop4.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(50,0,toRadians(90)),
                        getVelocityConstraint(55,toRadians(160),DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(55))
                .lineToSplineHeading(new Pose2d(50,27,toRadians(90)),
                        getVelocityConstraint(65,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(65))//55 120
                .build();

        Park2 = robot.drivetrain.trajectorySequenceBuilder(Drop4.end())
                .setReversed(false)
                .turn(toRadians(30))
                .lineToConstantHeading(new Vector2d(47.7,4.5),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();

        Park3 = robot.drivetrain.trajectorySequenceBuilder(Drop4.end())
                .setReversed(true)
                .splineTo(new Vector2d(48,-25),toRadians(270),
                        getVelocityConstraint(60,DriveConstants.MAX_ANG_VEL,DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(35))
                .build();


        while(!isStarted() && !isStopRequested())
        {
            robot.drivetrain.update();
            robot.vision.update();
            position = robot.vision.getPosition();


            telemetry.addLine("start");
            telemetry.addLine("Running 4 Cycle HIGH");
            telemetry.addData("position",robot.vision.getPosition());
            telemetry.addData("Voltage", voltageSensor.getVoltage());
            telemetry.update();
        }


        waitForStart();
        robot.vision.camera.stopStreaming();
        startIMUThread(this);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                        //////////////////////////////PRELOAD//////////////////////////////

                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(robot.drivetrain, Preload_drop),
                                new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.CLOSE)),
                                new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                                new WaitCommand(1360)
                                .andThen(new InstantCommand(()-> robot.lift.newProfile(HighJunctionPos)))
                        ),
                        new WaitCommand(350),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)),
                        new WaitCommand(400),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosIn)),
                        new WaitCommand(200),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosOut)),
                        new WaitCommand(200),


                        //////////////////////////////PICK1//////////////////////////////
                      /*  new ParallelCommandGroup(
                                new WaitCommand(1000)
                                        .andThen(new FollowTrajectoryCommand(robot.drivetrain, Pick1)),
                                new InstantCommand(()-> robot.intake.update(IntakeSubsystem.ClawState.CLOSE))
                                        .andThen(new WaitCommand(300))
                                .andThen(new InstantCommand(()-> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
                                                .andThen(new InstantCommand(()-> robot.lift.newProfile(0)))
                                                    .andThen(new WaitCommand(100))
                                                    .andThen( new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN))))
                                                        .andThen(new WaitCommand(700))
                                                            .andThen(new InstantCommand(()-> robot.intake.setFourbar(fourbarFIRSTPICK1,fourbarFIRSTPICK2)))
                        ),

                       */
                        new AutoPickCommand(robot,robot.drivetrain,Pick1,fourbarFIRSTPICK1,fourbarFIRSTPICK2),

                        new WaitCommand(150),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.CLOSE)),
                        new WaitCommand(150),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                        new WaitCommand(300),


                        //////////////////////////////DROP1//////////////////////////////
                        /*
                        new ParallelCommandGroup(
                                new FollowTrajectoryCommand(robot.drivetrain, Drop1),
                                new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                                new InstantCommand(() -> robot.lift.newProfile(HighJunctionPos)),
                                new WaitCommand(1450).andThen(new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                                ),
                        new WaitCommand(350),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosIn)),
                        new WaitCommand(250),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosOut)),
                        new WaitCommand(200),

                         */

                        new AutoDropCommand(robot, robot.drivetrain,Drop1),


                        //////////////////////////////PICK2//////////////////////////////
                        /*
                        new ParallelCommandGroup(
                                new WaitCommand(1000)
                                        .andThen(new FollowTrajectoryCommand(robot.drivetrain, Pick2)),
                                new InstantCommand(()-> robot.intake.update(IntakeSubsystem.ClawState.CLOSE))
                                        .andThen(new WaitCommand(250))
                                        .andThen(new InstantCommand(()-> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
                                                .andThen(new InstantCommand(()-> robot.lift.newProfile(0)))
                                                .andThen( new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN))))
                                                    .andThen(new WaitCommand(500))
                                            .andThen(new InstantCommand(()-> robot.intake.setFourbar(fourbarSECONDPICK1,fourbarSECONDPICK2)))
                        ),

                         */

                        new AutoPickCommand(robot, robot.drivetrain,Pick2,fourbarSECONDPICK1,fourbarSECONDPICK2),


                        new WaitCommand(150),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.CLOSE)),
                        new WaitCommand(150),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                        new WaitCommand(300),


                        //////////////////////////////DROP2//////////////////////////////

                      /*  new ParallelCommandGroup(
                                new FollowTrajectoryCommand(robot.drivetrain, Drop2),
                                new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                                new InstantCommand(() -> robot.lift.newProfile(HighJunctionPos)),
                                new WaitCommand(1250).andThen(new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new WaitCommand(350),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosIn)),
                        new WaitCommand(250),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosOut)),
                        new WaitCommand(200),

                       */

                        new AutoDropCommand(robot, robot.drivetrain,Drop2),


                        //////////////////////////////PICK3//////////////////////////////
                        /*
                        new ParallelCommandGroup(
                                new WaitCommand(1000)
                                        .andThen(new FollowTrajectoryCommand(robot.drivetrain, Pick3)),
                                new InstantCommand(()-> robot.intake.update(IntakeSubsystem.ClawState.CLOSE))
                                        .andThen(new WaitCommand(250))
                                .andThen(new InstantCommand(()-> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
                                                .andThen(new InstantCommand(()-> robot.lift.newProfile(0)))
                                                .andThen( new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN))))
                                                    .andThen(new WaitCommand(500))
                                                                .andThen(new InstantCommand(()-> robot.intake.setFourbar(fourbarTHIRDPICK1,fourbarTHIRDPICK2)))
                        ),

                         */

                        new AutoPickCommand(robot, robot.drivetrain,Pick3,fourbarTHIRDPICK1,fourbarTHIRDPICK2),

                        new WaitCommand(150),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.CLOSE)),
                        new WaitCommand(150),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                        new WaitCommand(300),


                        //////////////////////////////DROP3//////////////////////////////

                        /*new ParallelCommandGroup(
                                new FollowTrajectoryCommand(robot.drivetrain, Drop3),
                                new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                                new InstantCommand(() -> robot.lift.newProfile(HighJunctionPos)),
                                new WaitCommand(1450).andThen(new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new WaitCommand(350),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosIn)),
                        new WaitCommand(250),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosOut)),
                        new WaitCommand(200),

                         */

                        new AutoDropCommand(robot, robot.drivetrain,Drop3),

                        //////////////////////////////PICK4//////////////////////////////
                        /*
                        new ParallelCommandGroup(
                                new WaitCommand(1000)
                                        .andThen(new FollowTrajectoryCommand(robot.drivetrain, Pick4)),
                                new InstantCommand(()-> robot.intake.update(IntakeSubsystem.ClawState.CLOSE))
                                        .andThen(new WaitCommand(200))
                                        .andThen(new InstantCommand(()-> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
                                                .andThen(new InstantCommand(()-> robot.lift.newProfile(0)))
                                                .andThen( new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN))))
                                        .andThen(new WaitCommand(500))
                                        .andThen(new InstantCommand(()-> robot.intake.setFourbar(fourbarFOURTHPICK1,fourbarFOURTHPICK2)))
                        ),

                         */

                        new AutoPickCommand(robot, robot.drivetrain,Pick4,fourbarFOURTHPICK1,fourbarFOURTHPICK2),

                        new WaitCommand(150),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.CLOSE)),
                        new WaitCommand(150),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                        new WaitCommand(300),


                        //////////////////////////////DROP4//////////////////////////////

                        /*new ParallelCommandGroup(
                                new FollowTrajectoryCommand(robot.drivetrain, Drop4),
                                new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                                new InstantCommand(() -> robot.lift.newProfile(HighJunctionPos)),
                                new WaitCommand(1450).andThen(new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT)))
                        ),
                        new WaitCommand(350),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosIn)),
                        new WaitCommand(250),
                        new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                        new InstantCommand(()-> robot.lift.newProfile(HighJunctionPosOut)),
                        new WaitCommand(250),

                         */
                        new AutoDropCommand(robot, robot.drivetrain,Drop1),
                        //////////////////////////////PICK5//////////////////////////////

                        //////////////////////////////DROP5//////////////////////////////

                        //////////////////////////////PARK//////////////////////////////

                        new ParallelCommandGroup(
                                new WaitCommand(1000)
                                        .andThen(new FollowTrajectoryCommand(robot.drivetrain,position == 1? Park1 : position == 2?Park2 : Park3)),
                                new InstantCommand(()-> robot.intake.update(IntakeSubsystem.ClawState.CLOSE))
                                        .andThen(new WaitCommand(300))
                                            .andThen(new InstantCommand(()-> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
                                                .andThen(new InstantCommand(()-> robot.lift.newProfile(0)))
                                                    .andThen( new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN))))
                                                        .andThen(new InstantCommand(()->robot.intake.update(IntakeSubsystem.FourbarState.INTAKE)))

                        ),


                       new InstantCommand(this::requestOpModeStop)
                )

        );

        while(opModeIsActive())
        {

            CommandScheduler.getInstance().run();

            robot.lift.read();
            robot.lift.loop();
            robot.lift.write();
            robot.drivetrain.update();


            telemetry.addData("ticks",robot.lift.getDr4bPosition());
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
    public void update(int pos) {
        robot.lift.dr4b_motor.motor.setTargetPosition(pos);
        robot.lift.dr4b_motor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.dr4b_motor.set(1);
    }
}