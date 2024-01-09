package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Servo;

public class Servos {

    public static class DepositorClaw extends Servo {

        public DepositorClaw() {
            super(Hardware.depositorClaw);

        }

        public void open(){
            setPosition(0.5);
        }
        public void close(){
            setPosition(0.9);
        }
    }


    public static class DepositorV4B extends Servo {

        public DepositorV4B() {
            super(Hardware.v4b, Depositor.DepositorDash.v4bFront,0, Depositor.DepositorDash.v4bBack, 180,210,-30);
            Servo depositorV4b2 = new Servo(Hardware.v4b2);
            pair(depositorV4b2);
        }

        public void setPosition(double position){
            setPositionInterpolated(position);
        }
    }


    public static class DepositorClawTilt extends Servo{

        public DepositorClawTilt(){
            super(Hardware.clawTilt,0.35,-90,0.78,0);

            }

        public void setPosition(double position){
            setPositionInterpolated(position);
        }
    }

    public static class Turret extends Servo{
        public Turret(){
            super(Hardware.turret1,0.07,90,0.6,-45);
        }
        public void setPosition(double position){
            setPositionInterpolated(position);
        }

    }

    public static class PoleGuide extends Servo{
        public PoleGuide(){
            super(Hardware.guider);

        }

        public void down(){
            setPosition(Depositor.DepositorDash.guideDown);
        }

        public void up(){
            setPosition(Depositor.DepositorDash.guideUp);
        }
    }


}
