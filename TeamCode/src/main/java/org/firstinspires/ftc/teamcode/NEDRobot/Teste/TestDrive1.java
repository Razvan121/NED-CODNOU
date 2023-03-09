package org.firstinspires.ftc.teamcode.NEDRobot.Teste;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;

import java.text.DecimalFormat;

@TeleOp(name = "TestDrive1")
@Disabled
public class TestDrive1 extends LinearOpMode {

    private SampleMecanumDrive drive1;
    private double drive = 0, turn = 0, strafe = 0, speed = 0.6;
    private boolean last_left_bumper = false;
    private boolean last_right_bumper = false;
    private double powerFR, powerFL, powerRR, powerRL;

    private GamepadEx GamepadEx1;


    @Override
    public void runOpMode() throws InterruptedException {
       // CommandScheduler.getInstance().reset();
         drive1 = new SampleMecanumDrive(hardwareMap);
        GamepadEx1 = new GamepadEx(gamepad1);


        waitForStart();

        while (!isStopRequested()) {
            drive1.setWeightedDrivePower(
                    new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * 0.3,
                            dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * 0.3,
                            -GamepadEx1.getRightX()* 0.3
                    )
            );

            //takeControllerInput();
            //drive();
            boolean upOdo = gamepad1.y;
            boolean downOdo = gamepad1.a;

          /*  if (upOdo)
                robot.odometrySubsystem.update(OdometrySubsystem.OdoState.UP);
            if (downOdo)
                robot.odometrySubsystem.update(OdometrySubsystem.OdoState.DOWN);

           */
        }
    }


    private void takeControllerInput() {
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        if (gamepad1.left_bumper && last_left_bumper != gamepad1.left_bumper)
            speed = Math.max(0.15, speed - 0.15);
        if (gamepad1.right_bumper && last_right_bumper != gamepad1.right_bumper)
            speed = Math.min(1, speed + 0.15);

        last_left_bumper = gamepad1.left_bumper;
        last_right_bumper = gamepad1.right_bumper;
    }


    private void addTurn(double turn) {
        powerFR -= turn;
        powerRR -= turn;
        powerFL += turn;
        powerRL += turn;
    }


    private void drive() {

        powerFR = drive - strafe;
        powerFL = drive + strafe;
        powerRR = drive + strafe;
        powerRL = drive - strafe;

        addTurn(turn);

        // multiplies by speed
        powerFR *= speed;
        powerFL *= speed;
        powerRR *= speed;
        powerRL *= speed;

        // applies the power
        drive1.setMotorPowers(powerFL, powerRL, powerRR, powerFR);

        DecimalFormat df = new DecimalFormat("#%");
        telemetry.addData("speed", df.format(speed));
        telemetry.update();
    }
    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }

}
