package org.firstinspires.ftc.teamcode.NEDRobot.Teste;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config

public class TestOdo extends OpMode {
    public Servo servo_odo1,servo_odo2;
    public static double pos1=0,pos2=0;
    public double pos2_deposit=0.4,pos2_intake=0.55;
    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo_odo1 =  hardwareMap.get(Servo.class,"FrontOdo");
        servo_odo2 =  hardwareMap.get(Servo.class,"LeftOdo");
        servo_odo1.setDirection(Servo.Direction.REVERSE);

    }
    @Override
    public void loop(){
        servo_odo1.setPosition(pos1);
        servo_odo2.setPosition(pos2);
        //telemetry.addData("Pos1",intake1.getPosition());
        //telemetry.addData("Pos2",intake2.getPosition());
        telemetry.update();
    }
}
