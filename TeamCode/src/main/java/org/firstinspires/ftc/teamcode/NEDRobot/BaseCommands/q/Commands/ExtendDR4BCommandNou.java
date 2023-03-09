package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.Commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands.Dr4bGeneralCommand;
import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;

public class ExtendDR4BCommandNou extends ParallelCommandGroup {
    public ExtendDR4BCommandNou(Dr4bSubsystem dr4bSubsystem, int position){
        super(
                new Dr4bGeneralCommand(dr4bSubsystem,position,10000,10000,10, 1000, Dr4bSubsystem.STATE.FAILED_EXTEND)
        );
    }
}
