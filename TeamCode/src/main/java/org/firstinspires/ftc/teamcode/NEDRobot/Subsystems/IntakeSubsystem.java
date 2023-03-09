package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {
    public Servo claw;
    public Servo intake1;
    public Servo intake2;


    private double closePos=0.9;
    private double openPos=0.65;

  public static double intake_Pos1 = 0.76;//74
  public static double intake_Pos2 = 0.76;
  public static  double intakeTransitionIntake_Pos1 = 0.66;
  public static double intakeTransitionIntake_Pos2 = 0.66;
  public static double intakeTransitionDeposit_Pos1 =0.45;
  public static double intakeTransitionDeposit_Pos2 =0.45;
  public static double intakeDeposit_Pos1 =0.3;
  public static double intakeDeposit_Pos2 =0.3;
  public static double intakeJunction_Pos1 = 0.25;
  public static double intakeJunction_Pos2 = 0.25;

  public int offset = 0;
  public double offset2= 0;

    public FourbarState fourbarState = FourbarState.INTAKE;





    public enum ClawState {
        CLOSE,
        OPEN,
    }


    public enum FourbarState{
        INTAKE,
        TRANSITION_INTAKE,
        TRANSITION_DEPOSIT,
        DEPOSIT,
        JUNCTION
    }

    public IntakeSubsystem(HardwareMap hardwareMap,boolean isAuto) {
        this.claw = hardwareMap.get(Servo.class, "claw");
        this.intake1 = hardwareMap.get(Servo.class, "intake1");
        this.intake2 = hardwareMap.get(Servo.class, "intake2");
        this.intake2.setDirection(Servo.Direction.REVERSE);

        if(isAuto)
        {
            update(ClawState.CLOSE);
        }
        else
        {
            update(ClawState.OPEN);
            update(FourbarState.INTAKE);
        }
    }
    public void update(ClawState state)
    {
        switch (state){
            case CLOSE:
                claw.setPosition(closePos);
                break;
            case OPEN:
                claw.setPosition(openPos);
                break;
        }
    }
    public void update(FourbarState state)
    {
        fourbarState  = state;
        switch (state){
            case INTAKE:
                setFourbar(intake_Pos1,intake_Pos2);
                break;
            case TRANSITION_INTAKE:
                setFourbar(intakeTransitionIntake_Pos1,intakeTransitionIntake_Pos2);
                break;
            case TRANSITION_DEPOSIT:
                setFourbar(intakeTransitionDeposit_Pos1,intakeTransitionDeposit_Pos2);
                break;
            case DEPOSIT:
                setFourbar(intakeDeposit_Pos1,intakeDeposit_Pos2);
                break;
            case JUNCTION:
                setFourbar(intakeJunction_Pos1,intakeJunction_Pos2 );
                break;
        }
    }

    public void setIntakeFactor(double factor) {
        double IntakeAddition = 0.001 * factor;
        double IntakePos1 = getIntake1Position();
        double IntakePos2 = getIntake2Position();
            setFourbar(IntakePos1,IntakePos2);
    }

    public void setFourbar(double pos1, double pos2)
    {
        intake1.setPosition(pos1);
        intake2.setPosition(pos2);
    }


    public void adjustIntakeOffset( double offset)
    {
        this.offset2+= offset;
    }
    public double getClawPosition()
    {
        return claw.getPosition();
    }
    public double getIntake1Position()
    {
        return intake1.getPosition();
    }
    public double getIntake2Position()
    {
        return intake2.getPosition();
    }
    public double getAvgIntakePosition()
    {
        return (getIntake1Position()+getIntake2Position())/2;
    }
}