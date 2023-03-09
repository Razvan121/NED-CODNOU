package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;

public class TeleopIntakePos extends SequentialCommandGroup {
    public TeleopIntakePos(BaseRobot robot){
        super(
                new InstantCommand(()->robot.intakeSubsystem.update(IntakeSubsystem.ClawState.CLOSE)),
                new WaitCommand(150),
                new InstantCommand(() ->robot.intakeSubsystem.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
        );
    }
}
