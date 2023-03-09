package org.firstinspires.ftc.teamcode.NEDRobot.BaseCommands.q.GeneralCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.NEDRobot.Subsystems.Dr4bSubsystem;

public class Dr4bGeneralCommand extends CommandBase {
    private double position;
    private double timeout;
    private double max_v;
    private double max_a;
    private double max_d;
    private double allowed_error;

    private Dr4bSubsystem Dr4bSubsystem;
    private Dr4bSubsystem.STATE errorState;

    private ElapsedTime timer;

    public Dr4bGeneralCommand(Dr4bSubsystem Dr4bSubsystem, double position, double v, double a,
                              double allowed_error, double timeout, Dr4bSubsystem.STATE error)
    {
        this.Dr4bSubsystem = Dr4bSubsystem;
        this.position = position;
        this.max_v = v;
        this.max_a = a;
        this.max_d = 0;
        this.allowed_error = allowed_error;
        this.timeout = timeout;
        this.errorState = error;
    }

  /*  public Dr4bGeneralCommand(Dr4bSubsystem Dr4bSubsystem, double position, double v, double a, double d,
                              double allowed_error, double timeout, Dr4bSubsystem.STATE error)
    {
        this.Dr4bSubsystem = Dr4bSubsystem;
        this.position = position;
        this.max_v = v;
        this.max_a = a;
        this.max_d = d;
        this.allowed_error = allowed_error;
        this.timeout = timeout;
        this.errorState = error;
    }

   */

    @Override
    public void execute() {
        if (timer == null) {
            timer = new ElapsedTime();
            Dr4bSubsystem.newProfile(position, max_v, max_a);
        }

        if (timer.milliseconds() > timeout) {
            Dr4bSubsystem.liftstate= errorState;
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Dr4bSubsystem.getDr4bPosition() - position) < allowed_error || timer.milliseconds() > timeout;
    }
}
