package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.clawClosed;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.clawOpen;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.feedforward;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.slidesD;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.slidesDeadzone;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.slidesHomedConstant;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.slidesHomedThreshold;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.slidesI;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.slidesLowerLimit;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.slidesNormal;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.slidesP;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.slidesPosition;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.v4b1;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.v4b3;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.v4b4;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.v4b5;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.v4bDown;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.v4bFunny;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.v4bTransfer;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.velocityThreshold;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.wristFunny;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.wristNotFunny;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeDash.wristTransferPos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Motor;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Servo;

public class Intake extends Subsystem{

    @Config
    public static class IntakeDash{
        public static double wristPosition = .37;
        public static double slidesPosition = 800;
        public static double v4bPosition = .69;
        public static double clawPosition;
        public static double slidesP = 0.0038;
        public static double slidesI;
        public static double slidesD = 0.25;
        public static double slidesDeadzone;
        public static double slidesLowerLimit = 0.1;
        public static double slidesHomedThreshold = 30;
        public static double slidesHomedConstant = -0.1;
        public static double wristMax = .37;
        public static double wristMin = .1;
        public static double feedforward;
        public static double v4bDown = -38;
        public static double v4bFunny = 20;
        public static double v4bTransfer = 140;
        public static double clawOpen = 0.9;
        public static double clawClosed = 0.53;
        public static double wristNotFunny = 5;
        public static double wristTransferPos = 80;
        public static double wristFunny = -90;
        public static double slidesNormal = -10;
        public static double velocityThreshold = 30;
        public static double v4b5 = 10, v4b4 = 0, v4b3 = -10, v4b2 = -20, v4b1 = -38;
    }

    public Intake.IntakeState state = IntakeState.POST_TRANSFER;
    ElapsedTime stateTime = new ElapsedTime();
    Servo intakeClaw;
    Servo wrist;
    Servo v4b;
    Servo v4b2;
    Motor slides;
    Motor slides2;
    PID slidesPID;
    int stackHeight;
    public static boolean isReadyToTransfer = false;
    boolean transfer = false;

    public Intake() {
        intakeClaw = new Servo("claw");
        wrist = new Servo("clawpivot", 0.35,0, .08, -90,30,-120);
        v4b = new Servo("v4bleft",.49,90, .69, 0,145,-40);
        v4b2 = new Servo("v4bright", true);
        v4b.pair(v4b2);
        slides = new Motor(Hardware.intakeMotor1, false, true);
        slides2 = new Motor(Hardware.intakeMotor2, true);
        slides.pair(slides2);
        slidesPID = new PID(IntakeDash.slidesP,IntakeDash.slidesI,IntakeDash.slidesD);
        slidesPID.setDeadZone(IntakeDash.slidesDeadzone);
        slidesPID.setFeedForward(0);
        slidesPID.setLowerLimit(IntakeDash.slidesLowerLimit);
    }


    @Override
    public void update() {
        stateMachine();
        BaseOpMode.addData("state", state);
    }

    @Override
    public void updateSensors() {

    }
    public void setClawPosition(double position){
        intakeClaw.setPosition(position);
    }

    public void setv4bposition(double position){
        v4b.setPositionInterpolated(position);
    }
    public void setWristPosition(double position){
        wrist.setPositionInterpolated(position);
    }
    public double getSlidesPosition(){
        return slides.encoder.getPosition();
    }
    public void setSlidesPosition(double position){

        if(slides.encoder.getPosition()<0){
            slides.encoder.resetEncoder();
        }

        slidesPID.setConstants(slidesP,slidesI,slidesD);
        slidesPID.setDeadZone(slidesDeadzone);
        slidesPID.setFeedForward(feedforward);
        slidesPID.setLowerLimit(slidesLowerLimit);
        slidesPID.setHomedThreshold(slidesHomedThreshold);
        slidesPID.setHomedConstant(slidesHomedConstant);

        slides.setPower(slidesPID.getCorrection(slides.encoder.getPosition(), position));
        BaseOpMode.addData("intakeslides",slides.encoder.getPosition());

    }

    public void v4bStack(int height){
        switch(height){
            case 1:
                setv4bposition(v4b1);
                break;
            case 2:
                setv4bposition(IntakeDash.v4b2);
                break;
            case 3:
                setv4bposition(v4b3);
                break;
            case 4:
                setv4bposition(v4b4);
                break;
            case 5:
                setv4bposition(v4b5);
                break;
        }
    }

    public void decreaseStackHeight() {
        stackHeight --;
    }

    public void idleOpen(){
        setv4bposition(IntakeDash.v4bDown);
        setClawPosition(IntakeDash.clawOpen);
        setWristPosition(IntakeDash.wristNotFunny);
        setSlidesPosition(IntakeDash.slidesNormal);
    }
    public void idleClosed(){
        if(stateTime.seconds()<.8){
            setClawPosition(clawClosed);
            OpMode2.hasCone = true;
        }else {
            setv4bposition(v4bDown);
            setWristPosition(wristNotFunny);
            setSlidesPosition(IntakeDash.slidesNormal);
        }
    }
    public void transfer(){
        setv4bposition(v4bTransfer);
        setClawPosition(clawClosed);
        setWristPosition(wristNotFunny);
        setSlidesPosition(slidesNormal);
        if(stateTime.seconds() >.5){
            this.state = IntakeState.POST_TRANSFER;
            stateTime.reset();
        }
    }

    public void queueTransfer(){
        transfer();
        isReadyToTransfer = slides.encoder.getVelocity() <= velocityThreshold && slides.encoder.getPosition() <= slidesHomedThreshold;
        transfer = isReadyToTransfer && Depositor.isReadyToTransfer;
    }

    public void funny(){
        setv4bposition(v4bFunny);
        setClawPosition(clawOpen);
        setWristPosition(wristFunny);
        setSlidesPosition(slidesNormal);
    }

    public void slidesOut(){
        v4bStack(stackHeight);
        setClawPosition(clawOpen);
        setWristPosition(wristNotFunny);
        setSlidesPosition(slidesPosition);
        if(!OpMode2.hasCone){
            setState(IntakeState.IDLECLOSED);
        }
    }

    public void funnyClosed(){
        setv4bposition(v4bFunny);
        setClawPosition(clawClosed);
        setWristPosition(wristFunny);
        setSlidesPosition(slidesNormal);
    }
    public void transferSequence(){
        if(stateTime.seconds() > .4) {
            setWristPosition(wristTransferPos);
        }else if(stateTime.seconds()>.2){
            setClawPosition(clawOpen);
        }else {
            postTransfer();
            setClawPosition(clawClosed);
            setWristPosition(wristNotFunny);
        }
        if(stateTime.seconds() > 1.2){
            transfer = false;
            setState(IntakeState.POST_TRANSFER);
        }
    }

    public void postTransfer(){
        setv4bposition(v4bTransfer);
        setClawPosition(clawOpen);
        setSlidesPosition(slidesNormal);
    }

    public void home(){
        setSlidesPosition(0);
        setv4bposition(v4bTransfer);
        setClawPosition(clawClosed);
    }

    public void setState(Intake.IntakeState state){
        //if new state
        if(this.state != state && !transfer){
            this.state = state;
            isReadyToTransfer = false;
            stateTime.reset();
        }
    }
    public void stateMachine(){
        switch (state){
            case HOME:
                home();
                break;
            case IDLEOPEN:
                idleOpen();
                break;
            case IDLECLOSED:
                idleClosed();
                break;
            case TRANSFER:
                if(transfer) {
                    transferSequence();
                }else{
                    queueTransfer();
                }
                break;
            case FUNNY:
                funny();
                break;
            case POST_TRANSFER:
                postTransfer();
                break;
            case FUNNYCLOSED:
                funnyClosed();
                break;
            case SLIDESOUT:
                slidesOut();
                break;
        }
    }
    public enum IntakeState{
        FUNNY,IDLEOPEN,IDLECLOSED,TRANSFER, POST_TRANSFER, FUNNYCLOSED, SLIDESOUT, HOME
    }
}