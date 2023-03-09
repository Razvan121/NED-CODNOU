package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.Constraints;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.State;

@Config
@TeleOp(name="dr4b")
public class Testdr4b extends OpMode {

    public static double P=0.000;
    public static double I=0.0;
    public static double D=0.000;
    public static double max_v=10000;
    public static double max_a=10000;
    public static double max_d=10000;



    public AsymmetricMotionProfile profile;
    public State currentState;

    private ElapsedTime timer;
    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private PIDController controller;

    MotorEx dr4b_motor;

    private int dr4bPosition=0;
    public static double targetPosition=0;
    public static double rightcurrent=0;
    public double power=0;
    private double voltage;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dr4b_motor = new MotorEx(hardwareMap,"dr4b");


        MotorConfigurationType motorConfigurationType = dr4b_motor.motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        dr4b_motor.motor.setMotorType(motorConfigurationType);

        timer = new ElapsedTime();
        timer.reset();

        voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        profile = new AsymmetricMotionProfile(0,1,new Constraints(0,0,0));
        controller = new PIDController(P,I,D);
        controller.setPID(P,I,D);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
        dr4b_motor.resetEncoder();
    }
    @Override
    public void loop()
    {
        if(gamepad1.a)
        {
            profile = new AsymmetricMotionProfile(dr4bPosition, targetPosition, new Constraints(max_v, max_a, max_d));
            rightcurrent=targetPosition;
            timer.reset();
        }
        dr4bPosition=dr4b_motor.getCurrentPosition();

        controller = new PIDController(P,I,D);
        controller.setPID(P,I,D);

        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        currentState=profile.calculate(timer.time());
        if(currentState.v != 0)
        {
            rightcurrent= currentState.x;
        }

        power = controller.calculate(dr4bPosition,rightcurrent)/ voltage*12 ;

        dr4b_motor.set(power);


        telemetry.addData("dr4b_ticks",dr4bPosition);
        telemetry.addData("dr4b_ticks_target",targetPosition);
        telemetry.addData("time",timer.milliseconds());
        telemetry.addData("vel",dr4b_motor.getVelocity());
        telemetry.addData("power",power);

        telemetry.update();
    }

}
