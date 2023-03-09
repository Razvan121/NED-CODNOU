package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.NEDRobot.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.NEDRobot.trajectorysequence.TrajectorySequence;

public class FollowTrajectoryCommand extends CommandBase {
    private SampleMecanumDrive drive;
    private TrajectorySequence traj;

    public FollowTrajectoryCommand(SampleMecanumDrive sampleMecanumDrive, TrajectorySequence trajectorySequence) {
        drive = sampleMecanumDrive;
        traj = trajectorySequence;
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequenceAsync(traj);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }

    @Override
    public void end(boolean cancel) {
        if (cancel) {
            drive.followTrajectorySequenceAsync(null);
            drive.setDriveSignal(new DriveSignal());
        }
    }
}

