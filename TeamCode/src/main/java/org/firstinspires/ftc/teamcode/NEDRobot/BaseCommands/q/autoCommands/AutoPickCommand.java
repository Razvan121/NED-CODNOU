package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.autoCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobot;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.BaseRobotAuto;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;

public class AutoPickCommand extends ParallelCommandGroup {
    public AutoPickCommand(BaseRobotAuto robot, SampleMecanumDrive sampleMecanumDrive, TrajectorySequence trajectorySequence, double pos1, double pos2){
        super(
                new WaitCommand(1000)
                        .andThen(new FollowTrajectoryCommand(sampleMecanumDrive, trajectorySequence)),
                new InstantCommand(()-> robot.intake.update(IntakeSubsystem.ClawState.CLOSE))
                        .andThen(new WaitCommand(300))
                        .andThen(new InstantCommand(()-> robot.intake.update(IntakeSubsystem.FourbarState.TRANSITION_DEPOSIT))
                                .andThen(new InstantCommand(()-> robot.lift.newProfile(0)))
                                .andThen(new WaitCommand(100))
                                .andThen( new InstantCommand(()->robot.intake.update(IntakeSubsystem.ClawState.OPEN))))
                        .andThen(new WaitCommand(700))
                        .andThen(new InstantCommand(()-> robot.intake.setFourbar(pos1,pos2)))
        );
    }

}
