package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.autoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobotAuto;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;

public class AutoDropCommand extends ParallelCommandGroup {
    public AutoDropCommand(BaseRobotAuto robot, SampleMecanumDrive sampleMecanumDrive, TrajectorySequence trajectorySequence) {
        super(
                new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.CLOSE)),
                new FollowTrajectoryCommand(sampleMecanumDrive, trajectorySequence),

                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT)),
                new InstantCommand(() -> robot.lift.newProfile(1590)),
                new WaitCommand(1450).andThen(new InstantCommand(() -> robot.intake.update(IntakeSubsystem.FourbarState.DEPOSIT))),
                new WaitCommand(350),
                new InstantCommand(() -> robot.lift.newProfile(1420)),
                new WaitCommand(250),
                new InstantCommand(() -> robot.intake.update(IntakeSubsystem.ClawState.OPEN)),
                new InstantCommand(() -> robot.lift.newProfile(1590)),
                new WaitCommand(200)
            );
    }
}
