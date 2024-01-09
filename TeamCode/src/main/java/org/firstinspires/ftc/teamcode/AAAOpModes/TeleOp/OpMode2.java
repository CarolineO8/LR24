package org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths.Path.Path1;
import static org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths.Path.Path2;

import static org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths.Path.Path3;
import static org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths.Path.Score;
import static org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths.Path.Transfer;
import static org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths.Path.WaitForCone;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState.HOME;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState.SLIDESOUT;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState.TRANSFER;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.KCP.Movement;
import org.firstinspires.ftc.teamcode.Subsystems.Depositor;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp (name = "New OpMode 2")
public class OpMode2 extends BaseOpMode {

    Depositor depositor;
    Movement drive;

    Intake intake;

    public static boolean hasCone = true;

    Paths.Path state = Path1;

    boolean pathComplete = false;
    boolean transferComplete = false;




    @Override
    public void externalInit() {

        depositor = new Depositor();
        intake = new Intake();
        drive = new Movement(0, 0);


        Path1.compile();
        Path2.compile();

    }





    @Override
    public void externalLoop() {
//        drive.drive.directDrive(0,.5,0,0);

        stateMachine();

        BaseOpMode.addData("state",state);
    }

    public void stateMachine(){
        switch(state) {
            case Score:
                score();
                break;
            case WaitForCone:
                waitForCone();
                break;
            case Transfer:
                transfer();
                break;
            case Path1:
                path1();
                break;
            case Path2:
                path2();
                break;
            case Path3:
                path3();
                break;
        }

    }

    public void path1(){
        depositor.setState(Depositor.DepositorState.HOME);
        intake.setState(HOME);
        double heading = 0;
        if(Path1.t > .5){
            heading = 1.57;
        }
        if(!drive.followPath(Path1,.5,heading, .9,false)){
            setState(Path2);
        }
    }
    public void path2(){

        if(!drive.followPath(Path2,.5,1.57, .9,false)){
            setState(Path3);
        }
    }
    public void path3(){

        if(!pathComplete && drive.followPath(Path3,.5,1.57, 0,true)){
            intake.setState(TRANSFER);
            depositor.setState(Depositor.DepositorState.TRANSFER);
        }else{
            pathComplete = true;
            setState(Transfer);
        }
    }

    public void score(){
            depositor.setState(Depositor.DepositorState.AUTO_SCORE);
            if(depositor.getSlidesPos()>300){
                intake.setState(SLIDESOUT);
                setState(WaitForCone);
            }
            drive.holdPosition(0,0,0);

    }

    public void waitForCone(){
            if(intake.getSlidesPosition()< 50){
                setState(Transfer);
            }
        drive.holdPosition(0,0,0);
    }

    public void transfer(){
        if(depositor.getState() == Depositor.DepositorState.POST_TRANSFER){
            setState(Score);
        }else {
            intake.setState(TRANSFER);
            depositor.setState(Depositor.DepositorState.TRANSFER);
        }
        drive.holdPosition(0,0,0);
    }

    public void setState(Paths.Path state){
        this.state = state;
        pathComplete = false;
    }

}
