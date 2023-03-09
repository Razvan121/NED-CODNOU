package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands.ExtendDR4BCommandNou;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;

@Config
@TeleOp(name="dr4b14423")
public class Testdr4bGERF extends CommandOpMode {

    Dr4bSubsystem dr4bSubsystem;
    public static int targetPosition=0;
    @Override
    public void initialize() {
        dr4bSubsystem = new Dr4bSubsystem(hardwareMap, false);
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void run()
    {
        dr4bSubsystem.read();
        if(gamepad1.a)
        {
            schedule(new ExtendDR4BCommandNou(dr4bSubsystem,targetPosition));
        }
        CommandScheduler.getInstance().run();

        dr4bSubsystem.loop();
        dr4bSubsystem.write();
        telemetry.addData("dr4b_ticks",dr4bSubsystem.getDr4bPosition());
        telemetry.addData("dr4b_ticks_target",targetPosition);
        telemetry.addData("power",dr4bSubsystem.dr4b_motor.get());

        telemetry.update();
    }

    public void reset()
    {
        dr4bSubsystem.dr4b_motor.resetEncoder();
    }
}
