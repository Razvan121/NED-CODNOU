package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Dr4bSubsystem extends SubsystemBase {

    public STATE liftstate= STATE.GOOD;
    public final MotorEx dr4b_motor;
    public enum STATE{
        GOOD,
        FAILED_EXTEND,
        FAILED_RETRACT
    }

    public MotionProfile profile;
    public MotionState currentState;
    public static double P=0.02;//0.02
    public static double I=0;
    public static double D=0.0003;//0.0003;
    public static double F=0.00025;//.00025;

    private ElapsedTime timer;
    private ElapsedTime voltageTimer;
    private VoltageSensor voltageSensor;
    private PIDFController controller;

    private int dr4bPosition=0;
    public double targetPosition=0;
    public double power=0;
    private double voltage;

    public Dr4bSubsystem(HardwareMap hardwareMap, boolean isAuto) {
        this.dr4b_motor = new MotorEx(hardwareMap,"dr4b");
        if(isAuto)
        {
            this.dr4b_motor.resetEncoder();
        }
        this.timer = new ElapsedTime();
        timer.reset();

        this.voltageTimer = new ElapsedTime();
        voltageTimer.reset();

            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1,0), new MotionState(0,0), 100,100);

       // this.profile = new MotionProfile(0,1,new Constraints(0,0,0));
        this.controller = new PIDFController(P,I,D,F);
        controller.setPIDF(P,I,D,F);

        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
    }

    public void loop()
    {
        controller = new PIDFController(P,I,D,F);
        if (voltageTimer.seconds() > 5) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
      currentState=profile.get(timer.time());
        if (currentState.getV() != 0)
        {
            targetPosition= currentState.getX();
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

    public void setDr4bFactor(double factor) {
        double dr4bAddition = 5 * factor;
        double newPosition = dr4bPosition + dr4bAddition;
        if (currentState.getV() == 0) {
            targetPosition = newPosition;
        }
    }

    public void resetTimer(){
        timer.reset();
    }

    public void newProfile(double targetPos, double max_v, double max_a){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(getDr4bPosition(),0),new MotionState(targetPos,0),max_v,max_a);
        resetTimer();
    }
  /*  public void newProfile(double targetPos, double max_v, double max_a) {
            this.newProfile(targetPos, max_v, max_a, max_a);
        }


        public void newProfile(double targetPos, double max_v, double max_a, double max_d) {
            profile = new AsymmetricMotionProfile(getDr4bPosition(), targetPos, new Constraints(max_v, max_a, max_d));
            resetTimer();
        }


   */
}