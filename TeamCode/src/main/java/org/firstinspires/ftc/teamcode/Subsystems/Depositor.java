package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.OpMode2;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.MotorEncoder;

public class Depositor extends Subsystem{


    @Config
    public static class DepositorDash{
        public static double clawTiltPos = 40;
        public static double v4bFront = 0.65, v4bBack = 0.23;
        public static double slideP=0.008, slideI, slideD=0.4;
        public static double slideFeedForward = 0.1, slideLowerLimit=0.09, slideDeadzone, slideHomedThreshold=50, slideHomedConstant=-0.2;
        public static int slidesHighGoal = 650, slidesMidGoal = 100, slidesLowGoal = 0;
        public static double v4bPos = 100;
        public static double v4bPosLow = 100;
        public static double v4bTransferGrabPos = -20;
        public static int highMidLow = 1;
        public static double v4bTransferPos = 20;
        public static double slideVelocityStopped = 30;
        public static double guideDown = 0.7, guideUp = 0.5;
        public static double autoTurret = 75, autoSlides = 700, autoV4b = 130;


    }

    DepositorState state = DepositorState.HOME;
    ElapsedTime stateTime = new ElapsedTime();
    PID slidesPid;
    Servos.DepositorClaw claw;
    Servos.DepositorClawTilt clawTilt;
    Servos.DepositorV4B v4b;
    Servos.PoleGuide guide;
    Motor slides;
    Motor slides2;
    MotorEncoder slidesEncoder;
    Servos.Turret turret;
    boolean transfer = false;
    public static boolean isReadyToTransfer = false;
    int setSlides = 0;

    public Depositor(){
        super();
        slidesPid = new PID(DepositorDash.slideP,DepositorDash.slideI, DepositorDash.slideD);
        slidesPid.setDeadZone(DepositorDash.slideDeadzone);
        slidesPid.setFeedForward(DepositorDash.slideFeedForward);
        slidesPid.setLowerLimit(DepositorDash.slideLowerLimit);

        slides = new Motor(Hardware.liftMotor1, false, true);
        slides2 = new Motor(Hardware.liftMotor2, true);
        slides.pair(slides2);

        slidesEncoder = slides.encoder;

        claw = new Servos.DepositorClaw();
        clawTilt = new Servos.DepositorClawTilt();
        v4b = new Servos.DepositorV4B();
        guide = new Servos.PoleGuide();

        turret = new Servos.Turret();

    }



    public void updateSlides(int position){

        if(slidesEncoder.getPosition()<0){
            slidesEncoder.resetEncoder();
        }

        slidesPid.setConstants(DepositorDash.slideP,DepositorDash.slideI, DepositorDash.slideD);
        slidesPid.setDeadZone(DepositorDash.slideDeadzone);
        slidesPid.setFeedForward(DepositorDash.slideFeedForward);
        slidesPid.setLowerLimit(DepositorDash.slideLowerLimit);
        slidesPid.setHomedThreshold(DepositorDash.slideHomedThreshold);
        slidesPid.setHomedConstant(DepositorDash.slideHomedConstant);
        slides.setPower(slidesPid.getCorrection(slidesEncoder.getPosition(), position));
        BaseOpMode.addData("Slides Encoder", slidesEncoder.getPosition());
    }

    public void setSlidesPos(int position){
        setSlides = position;
    }

    public double getSlidesPos(){
        return slidesEncoder.getPosition();
    }

    public void setTurretPos(double position){
        turret.setPosition(position);
    }




    @Override
    public void update() {
        stateMachine();
        updateSlides(setSlides);
        BaseOpMode.addData("state", state);
        BaseOpMode.addData("Transfer", transfer);
    }

    @Override
    public void updateSensors() {

    }


    public void home(){
        guide.up();

            setSlidesPos(0);
            v4b.setPosition(DepositorDash.v4bTransferPos);
            claw.open();
            clawTilt.setPosition(0);
            turret.setPosition(0);
    }

    public void highGoal(){
        guide.down();
        setSlidesPos(DepositorDash.slidesHighGoal);
        v4b.setPosition(DepositorDash.v4bPos);
        claw.close();
        clawTilt.setPosition(DepositorDash.clawTiltPos);
        turret.setPosition(0);
    }
    public void midGoal(){
        guide.down();
        setSlidesPos(DepositorDash.slidesMidGoal);
        v4b.setPosition(DepositorDash.v4bPos);
        claw.close();
        clawTilt.setPosition(DepositorDash.clawTiltPos);
        turret.setPosition(0);
    }
    public void lowGoal(){
        guide.down();
        setSlidesPos(0);
        v4b.setPosition(DepositorDash.v4bPosLow);
        claw.close();
        clawTilt.setPosition(DepositorDash.clawTiltPos);
        turret.setPosition(0);
    }

    public void transferSequence(){
        setSlidesPos(0);
        turret.setPosition(0);
        if(stateTime.seconds()>0.5) {
            v4b.setPosition(DepositorDash.v4bTransferGrabPos);
            if(stateTime.seconds()>0.8) {
                claw.close();
            }
        }else{
            claw.open();
        }

        clawTilt.setPosition(0);
        if(stateTime.seconds() > 1.2){
            transfer = false;
            setState(DepositorState.POST_TRANSFER);
        }
    }
    public void postTransfer(){
        turret.setPosition(0);
        setSlidesPos(0);
        v4b.setPosition(DepositorDash.v4bTransferPos);
        claw.close();
        clawTilt.setPosition(0);
    }
    public void queueTransfer(){
        home();
        isReadyToTransfer = slidesEncoder.getVelocity() <= DepositorDash.slideVelocityStopped && slidesEncoder.getPosition() <= DepositorDash.slideHomedThreshold;
        transfer = Intake.isReadyToTransfer && isReadyToTransfer;
    }

    public void scoreSequence(){
        if(slidesEncoder.getVelocity() <= DepositorDash.slideVelocityStopped){
            claw.open();
            OpMode2.hasCone = false;
            if(stateTime.seconds()>0.5){
                v4b.setPosition(DepositorDash.v4bTransferPos);
                claw.open();
                clawTilt.setPosition(0);
                turret.setPosition(0);
                if(stateTime.seconds()>1) {
                    setState(DepositorState.HOME);
                }
            }
        }else{
            stateTime.reset();
        }
    }

    public void autoScore(){
        setSlidesPos((int)DepositorDash.autoSlides);
        if(stateTime.seconds()>.5) {
            v4b.setPosition(DepositorDash.autoV4b);
        }else{
            v4b.setPosition(90);
        }
        if(stateTime.seconds()>.2) {
            turret.setPosition(DepositorDash.autoTurret);
        }else{
            turret.setPosition(0);
        }
        clawTilt.setPosition(DepositorDash.clawTiltPos);
        if(stateTime.seconds()>0.6){
            setState(DepositorState.SCORE);
        }
            claw.close();
    }


    public void setState(DepositorState state){

        if(this.state != state && !transfer){
            if(state != DepositorState.SCORE ) {
                this.state = state;
                isReadyToTransfer = false;
                stateTime.reset();
            }else if(this.state == DepositorState.HIGH || this.state == DepositorState.MID || this.state == DepositorState.LOW || this.state == DepositorState.AUTO_SCORE){
                    this.state = state;
                    stateTime.reset();
            }
        }
    }

    public DepositorState getState(){
        return state;
    }

    public void stateMachine(){
        switch (state){
            case LOW:
                lowGoal();
                break;
            case MID:
                midGoal();
                break;
            case HIGH:
                highGoal();
                break;
            default:
            case HOME:
                home();
                break;
            case TRANSFER:
                if(transfer) {
                    transferSequence();
                }else{
                    queueTransfer();
                }
                break;
            case POST_TRANSFER:
                postTransfer();
                break;
            case SCORE:
                    scoreSequence();
                break;
            case AUTO_SCORE:
                autoScore();
                break;
        }
    }

    public enum DepositorState{
        HIGH, MID, LOW, TRANSFER, HOME, POST_TRANSFER, FRONT_LOW, SCORE, AUTO_SCORE
    }






}
