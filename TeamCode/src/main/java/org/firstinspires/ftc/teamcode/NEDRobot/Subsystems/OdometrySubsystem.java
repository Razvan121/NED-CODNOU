    package org.firstinspires.ftc.teamcode.NEDRobot.Subsystems;

    import com.acmerobotics.dashboard.config.Config;
    import com.arcrobotics.ftclib.command.SubsystemBase;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.hardware.Servo;

    @Config
    public class OdometrySubsystem extends SubsystemBase {

        private Servo LeftOdo,FrontOdo;
        public enum OdoState {
            UP,
            DOWN
        }
        private double odo_up_pos1=0.52,odo_up_pos2=0.55;
        private double odo_down_pos1=0.6,odo_down_pos2=0.47;

        public OdometrySubsystem(HardwareMap hardwareMap,boolean isAuto){
            this.LeftOdo = hardwareMap.get(Servo.class,"LeftOdo");
            this.FrontOdo = hardwareMap.get(Servo.class,"FrontOdo");
            this.FrontOdo.setDirection(Servo.Direction.REVERSE);

            if(isAuto)
            {
                update(OdoState.UP);
                update(OdoState.DOWN);
            }
            else
            {
                update(OdoState.UP);
            }
        }

        public void update(OdoState state)
        {
            switch (state){
                case UP:
                    setOdo(odo_up_pos1,odo_up_pos2);
                    break;
                case DOWN:
                    setOdo(odo_down_pos1,odo_down_pos2);
                    break;
            }
        }
        public void setOdo(double pos1,double pos2)
        {
            FrontOdo.setPosition(pos1);
            LeftOdo.setPosition(pos2);
        }
    }
