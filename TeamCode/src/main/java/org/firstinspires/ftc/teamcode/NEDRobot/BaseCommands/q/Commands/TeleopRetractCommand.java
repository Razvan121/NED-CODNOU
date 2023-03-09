package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

public class TeleopRetractCommand extends ParallelCommandGroup {
    public TeleopRetractCommand(BaseRobot robot){
        super(
                new InstantCommand(()-> robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                new WaitCommand(350),
                new InstantCommand(() -> robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                new WaitCommand(600),
                new RetractDR4BCommand(robot,0)
        );
    }
}
