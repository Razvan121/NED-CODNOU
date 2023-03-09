package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config

public class TestMotor extends OpMode {

    public DcMotor motor;
    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotor.class,"motor");

    }
    @Override
    public void loop(){
        if(-gamepad1.left_stick_y>0.3)
            motor.setPower(1);
        else
            if(-gamepad1.left_stick_y<-0.3)
                motor.setPower(-1);
            else
                motor.setPower(0);
        telemetry.update();
    }
}
