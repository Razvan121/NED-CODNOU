package org.firstinspires.ftc.teamcode.NEDRobot.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.TeleopDepositCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.TeleopIntakePos;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.TeleopRetractCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.TeleopExtendDR4BCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;


@Config
@TeleOp(name = "OpModeNational", group = "National")
public class CommandNational extends CommandOpMode {

    private double loopTime = 0;
    private BaseRobot robot;

    private  int HighJunctionPos =  1630;
    private  int MidJunctionPos = 1000;
    private  int LowJunctionPos = 490;


    private boolean NUSTRICAINTAKE = false;

    private InstantCommand closeClawCommand;
    private InstantCommand openClawCommand;
    private InstantCommand FourBarIntakeCommand;
    private InstantCommand FourBarDepositCommand;
    private InstantCommand FourBarTransitionIntakeCommand;
    private InstantCommand FourBarJunctionCommand;
    private InstantCommand FourBarTransitionDepositCommand;

    public boolean Scoring;


    private ElapsedTime timer;
    private GamepadEx GamepadEx1;

    boolean stackButton = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robot = new BaseRobot(hardwareMap,false);
        robot.reset();





        robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.INTAKE);

        FourBarIntakeCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.INTAKE));

        FourBarDepositCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.DEPOSIT));

        FourBarTransitionIntakeCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_INTAKE));

        FourBarJunctionCommand = new InstantCommand(()-> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.JUNCTION));

        FourBarTransitionDepositCommand  = new InstantCommand(()->robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT));

        openClawCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.OPEN));

        closeClawCommand = new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE));

        GamepadEx1 = new GamepadEx(gamepad1);
        //GamepadEx2 = new GamepadEx(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        PhotonCore.enable();


    }
    @Override
    public void run() {


        robot.read();

        //////////////////////////////GAMEPAD1//////////////////////////////////////////////////////////

        if(Scoring) {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * 0.3,
                            dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * 0.3,
                            -GamepadEx1.getRightX()* 0.3
                    )
            );
        }
        else
        {
            robot.drive.setWeightedDrivePower(
                    new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * (gamepad1.left_trigger > 0.5 ? 0.3 : 1),
                            dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * (gamepad1.left_trigger > 0.5 ? 0.3 : 1),
                            -GamepadEx1.getRightX() * (gamepad1.left_trigger > 0.5 ? 0.3 : 1)
                    )
            );
        }

        robot.drive.setWeightedDrivePower(
                new Pose2d(dead(scale(GamepadEx1.getLeftY(), 0.6), 0) * (gamepad1.left_trigger > 0.5 ? 0.3 : 1),
                        dead(-scale(GamepadEx1.getLeftX(), 0.6), 0) * (gamepad1.left_trigger > 0.5 ? 0.3 : 1),
                        (robot.dr4bSubsystem.isExtended()? -scale(GamepadEx1.getRightX(),0.6) *0.3 :-scale(GamepadEx1.getRightX(),0.6)* (gamepad1.left_trigger > 0.5 ? 0.3 : 1))
                )
        );


        if(gamepad1.left_bumper)
        {
            CommandScheduler.getInstance().schedule(new TeleopIntakePos(robot));
        }
        if(gamepad1.right_bumper)
        {
            CommandScheduler.getInstance().schedule(new TeleopDepositCommand(robot));
            NUSTRICAINTAKE = false;
            Scoring=false;

        }

        if(gamepad1.right_trigger>0.5)
        {
            schedule(FourBarJunctionCommand);
        }


        if(gamepad1.y)
        {
            schedule(new TeleopExtendDR4BCommand(robot,HighJunctionPos));
            Scoring=true;
            NUSTRICAINTAKE  = true;
        }
        if(gamepad1.x)
        {
            schedule(new TeleopExtendDR4BCommand(robot,MidJunctionPos));
            Scoring=true;
            NUSTRICAINTAKE =  true;
        }

        if(gamepad1.b)
        {
            schedule(new TeleopExtendDR4BCommand(robot, LowJunctionPos));
            Scoring=true;
            NUSTRICAINTAKE =  true;
        }
        if(gamepad1.a)
        {
            schedule(new TeleopRetractCommand(robot));
            Scoring = false;
            NUSTRICAINTAKE =  false;
        }

        boolean d1DU = gamepad1.dpad_up;
        boolean d1DD = gamepad1.dpad_down;
        boolean d1DL = gamepad1.dpad_left;
        boolean d1DR = gamepad1.dpad_right;

        if(d1DU && !d1DR && !d1DL) {
            schedule(FourBarDepositCommand);

        }

        if(d1DD && !d1DR && !d1DL && !NUSTRICAINTAKE ) {
            schedule(FourBarIntakeCommand);

        }


        if(d1DL && !d1DU && !d1DD  ) {
            schedule(FourBarTransitionIntakeCommand);

        }

        if(d1DR && !d1DU && !d1DD ) {
            schedule(FourBarTransitionDepositCommand);

        }


        //////////////////////////////GAMEPAD2//////////////////////////////////////////////////////////


        boolean d2DU = gamepad2.dpad_up;
        boolean d2DD = gamepad2.dpad_down;
        boolean d2DL = gamepad2.dpad_left;
        boolean d2DR = gamepad2.dpad_right;

       // boolean stackButton = false;

        stackButton  = gamepad2.options;

        if(d2DU && !d2DR && !d2DL && !stackButton) {
            schedule(FourBarDepositCommand);

        }

        if(d2DD && !d2DR && !d2DL && !NUSTRICAINTAKE && !stackButton) {
            schedule(FourBarIntakeCommand);

        }


        if(d2DL && !d2DU && !d2DD && !stackButton) {
            schedule(FourBarTransitionIntakeCommand);

        }

        if(d2DR && !d2DU && !d2DD && !stackButton) {
            schedule(FourBarTransitionDepositCommand);

        }

        ///////////// stack////////////////////////////////////

        if(d2DU && !d2DR && !d2DL && stackButton) {
            robot.intakeSubsystem.setFourbar(0.66,0.66);

        }

        if(d2DR && !d2DU && !d2DL && !NUSTRICAINTAKE && stackButton) {
            robot.intakeSubsystem.setFourbar(0.69,0.69);

        }

        if(d2DL && !d2DR && !d2DD && stackButton) {
            robot.intakeSubsystem.setFourbar(0.71,0.71);

        }


        if(d2DD && !d2DR && !d2DL && stackButton) {
            robot.intakeSubsystem.setFourbar(0.75,0.75);

        }



        if(gamepad2.left_bumper) {
            schedule(closeClawCommand);
        }


        if(gamepad2.right_bumper) {
            schedule(openClawCommand);
        }

        if(gamepad2.right_trigger>0.5)
        {
            schedule(FourBarJunctionCommand);
        }

        if(gamepad2.left_trigger>0.5)
        {
            schedule(FourBarDepositCommand);
        }

        if(gamepad2.y)
        {
            schedule(new TeleopExtendDR4BCommand(robot,HighJunctionPos));
            Scoring=true;
            NUSTRICAINTAKE  = true;
        }
        if(gamepad2.x)
        {
            schedule(new TeleopExtendDR4BCommand(robot,MidJunctionPos));
            Scoring=true;
            NUSTRICAINTAKE =  true;
        }

        if(gamepad2.b)
        {
            schedule(new TeleopExtendDR4BCommand(robot, LowJunctionPos));
            Scoring=true;
            NUSTRICAINTAKE =  true;
        }
        if(gamepad2.a)
        {
            schedule(new TeleopRetractCommand(robot));
            Scoring = false;
            NUSTRICAINTAKE =  false;
        }


        if(Math.abs(gamepad2.left_stick_x)>0.3)
        {
            robot.dr4bSubsystem.setDr4bFactor(Math.pow(gamepad2.left_stick_x,3));
        }
        if(Math.abs(gamepad2.right_stick_x)>0.3)
        {
            robot.intakeSubsystem.setIntakeFactor(Math.pow(gamepad2.right_stick_x,3));
        }

        robot.dr4bSubsystem.loop();

        CommandScheduler.getInstance().run();

        robot.write();

        telemetry.addData("dr4b_ticks",robot.dr4bSubsystem.getDr4bPosition());
        telemetry.addData("dr4b_velocity",robot.dr4bSubsystem.dr4b_motor.getVelocity());
        telemetry.addData("dr4b_target_pos",robot.dr4bSubsystem.getLiftTargetPosition());
        telemetry.addData("dr4b_power",robot.dr4bSubsystem.dr4b_motor.get());
        telemetry.addData("intake1_pos",robot.intakeSubsystem.getIntake1Position());
        telemetry.addData("intake2_pos",robot.intakeSubsystem.getIntake2Position());
        telemetry.addData("intakeAVG_pos",robot.intakeSubsystem.getAvgIntakePosition());
        telemetry.addData("claw_pos",robot.intakeSubsystem.getClawPosition());
        telemetry.addData("dr4b_state", robot.dr4bSubsystem.liftstate);



        double loop = System.nanoTime();

        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();


    }
    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        robot.dr4bSubsystem.dr4b_motor.resetEncoder();
        robot.intakeSubsystem.intake1.setPosition(0);
        robot.intakeSubsystem.intake2.setPosition(0);
        robot.intakeSubsystem.claw.setPosition(0);
    }



    public double scale(double x, double k) {
        return (1 - k) * x + k * x * x * x;
    }

    public double dead(double x, double k) {
        return Math.abs(x) > k ? x : 0;
    }

}
