package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import androidx.annotation.GuardedBy;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;

public class BaseRobot {
    public OdometrySubsystem odometrySubsystem;
    public IntakeSubsystem intakeSubsystem;
    public Dr4bSubsystem dr4bSubsystem;
    public SampleMecanumDrive drive;
    private boolean isAuto;

    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public BNO055IMU imu;
    private double imuAngle = 0;
    private Thread imuThread;
    public BaseRobot(HardwareMap hardwareMap,boolean isAuto) {
        this.isAuto = isAuto;


        synchronized (imuLock) {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
        }

        this.odometrySubsystem = new OdometrySubsystem(hardwareMap, isAuto);
        this.intakeSubsystem = new IntakeSubsystem(hardwareMap, isAuto);
        this.dr4bSubsystem = new Dr4bSubsystem(hardwareMap, isAuto);
        this.drive = new SampleMecanumDrive(hardwareMap);

    }
      public BaseRobot(HardwareMap hardwareMap){
            this(hardwareMap,false);
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
        dr4bSubsystem.dr4b_motor.resetEncoder();
    }
    public void read()
    {
        dr4bSubsystem.read();
    }
    public void write()
    {
        dr4bSubsystem.write();
    }


}
