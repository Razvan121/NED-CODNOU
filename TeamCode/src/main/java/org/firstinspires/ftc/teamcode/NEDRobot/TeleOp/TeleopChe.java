package org.firstinspires.ftc.teamcode.NEDRobot.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;

public class TeleopChe extends LinearOpMode {

    public DcMotorEx rightSlide, leftSlide;
    public SampleMecanumDrive sampleMecanumDrive;

    public Servo leftIntake, rightIntake;
    public Servo wrist;
    public Servo claw;


    boolean turnWrist = false;

    public static double clawInit;
    public static double intakeIn;
    public static double intakeOut;
    public static double intakeMid;
    public static double clawClose;
    public static double clawOpen;
    public static double wristPut;
    public static double wristTake;


    @Override
    public void runOpMode() throws InterruptedException {
        sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);

        leftIntake = hardwareMap.get(Servo.class,"leftIntake");
        rightIntake = hardwareMap.get(Servo.class,"rightIntake");
        wrist = hardwareMap.get(Servo.class,"wrist");
        claw = hardwareMap.get(Servo.class,"intakeClaw");

        leftSlide = hardwareMap.get(DcMotorEx.class,"leftLiftMotor");
        rightSlide = hardwareMap.get(DcMotorEx.class,"rightLiftMotor");

        waitForStart();

        claw.setPosition(clawClose);
        intakePos(intakeIn);



        while(!isStopRequested())
        {
            if(gamepad2.right_bumper)
                claw.setPosition(clawOpen);
            if(gamepad2.left_bumper)
                claw.setPosition(clawClose);

            if(gamepad2.y) {
                intakePos(intakeIn);
                turnWrist = false;
            }

            if(gamepad2.a)///jos
            {
                intakePos(intakeOut);
                turnWrist =  true;
            }

            if(gamepad2.b && turnWrist==true)
            {
                wrist.setPosition(wristPut);
            }
            if(gamepad2.x)
            {
                wrist.setPosition(wristTake);
            }


        }

    }


    public void intakePos(double pos1)
    {
        leftIntake.setPosition(pos1);
        rightIntake.setPosition(pos1);
    }

}
