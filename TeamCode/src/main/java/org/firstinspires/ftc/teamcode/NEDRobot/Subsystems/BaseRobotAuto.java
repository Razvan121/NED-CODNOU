package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NEDRobot.Vision.Vision;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;

public class BaseRobotAuto {
    public OdometrySubsystem odometry;
    public IntakeSubsystem intake;
    public Dr4bAutoSubsystem lift;
    public SampleMecanumDrive drivetrain;


    public Vision vision;
    private boolean isAuto;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private double imuAngle = 0;
    private Thread imuThread;


    public BaseRobotAuto(HardwareMap hardwareMap) {
        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

        odometry = new OdometrySubsystem(hardwareMap, true);
        intake = new IntakeSubsystem(hardwareMap, true);
        lift = new Dr4bAutoSubsystem(hardwareMap);
        drivetrain = new SampleMecanumDrive(hardwareMap);
        vision = new Vision(hardwareMap);

    }
    public void init()
    {
        intake.update(IntakeSubsystem.ClawState.CLOSE);
        odometry.update(OdometrySubsystem.OdoState.UP);
        odometry.update(OdometrySubsystem.OdoState.DOWN);
        intake.update(IntakeSubsystem.FourbarState.TRANSITION_INTAKE);
        lift.dr4b_motor.resetEncoder();
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

    public double getAngle() {
        return imuAngle;
    }

    public void reset()
    {
        lift.dr4b_motor.resetEncoder();
    }
    public void read()
    {
        lift.read();
    }
    public void write(){lift.write();}


}
