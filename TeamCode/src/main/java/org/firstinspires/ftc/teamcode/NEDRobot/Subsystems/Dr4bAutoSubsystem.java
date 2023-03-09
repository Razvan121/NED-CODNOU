package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.Constraints;
import org.firstinspires.ftc.teamcode.NEDRobot.utilMotion.State;

@Config
public class Dr4bAutoSubsystem extends SubsystemBase {

    public STATE liftstate= STATE.GOOD;
    public final MotorEx dr4b_motor;
    public enum STATE{
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }


    public AsymmetricMotionProfile profile;
    public State currentState;
    public static double P=0.006;//0.02
    public static double I=0;
    public static double D=0.0000;
    private ElapsedTime timer;
    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private PIDController controller;

    private int dr4bPosition=0;
    public double targetPosition=0;
    public double power=0;
    private double voltage;

    public MotorConfigurationType motorConfigurationType;

    public Dr4bAutoSubsystem(HardwareMap hardwareMap) {
        this.dr4b_motor = new MotorEx(hardwareMap,"dr4b");

        this.motorConfigurationType = this.dr4b_motor.motor.getMotorType().clone();
        this.motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        this.dr4b_motor.motor.setMotorType(motorConfigurationType);

        this.dr4b_motor.resetEncoder();
        this.timer = new ElapsedTime();
        timer.reset();

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

        this.profile = new AsymmetricMotionProfile(0,1,new Constraints(0,0,0));


        this.controller = new PIDController(P,I,D);
        controller.setPID(P,I,D);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
    }

    public void loop()
    {
        controller = new PIDController(P,I,D);
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        currentState=profile.calculate(timer.time());
        if (currentState.v != 0)
        {
            targetPosition= currentState.x;
        }


        power = controller.calculate(dr4bPosition,targetPosition)/ voltage*12 ;
    }
    public void read() {
        dr4bPosition=dr4b_motor.getCurrentPosition();
    }

    public void write()
    {
        dr4b_motor.set(power);
    }

    public int getDr4bPosition()
    {
        return dr4bPosition;
    }

    public double getLiftTargetPosition()
    {
        return targetPosition;
    }
    public void resetTimer(){
        timer.reset();
    }

    public void newProfile(double targetPos){
        profile = new AsymmetricMotionProfile(getDr4bPosition(),targetPos, new Constraints(4800, 4800, 5500));
        resetTimer();
    }

}