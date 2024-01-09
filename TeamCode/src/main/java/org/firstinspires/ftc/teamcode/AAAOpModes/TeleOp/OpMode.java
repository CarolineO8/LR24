package org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState.FUNNY;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState.IDLEOPEN;
import static org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeState.SLIDESOUT;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Depositor;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Gyro;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp (name = "New OpMode")
public class OpMode extends BaseOpMode {

    Depositor depositor;
    Drivetrain drivetrain;
    Gyro gyro;


    Intake intake;

    @Override
    public void externalInit() {
        drivetrain = new Drivetrain();

        depositor = new Depositor();
        intake = new Intake();
    }


    @Override
    public void externalLoop() {
        ;

        drivetrain.drive(driver1.leftStick.Y(), driver1.leftStick.X(), driver1.rightStick.X());


        if (driver1.square.isTapped()) {
            intake.setState(Intake.IntakeState.TRANSFER);
            depositor.setState(Depositor.DepositorState.TRANSFER);
        } else if (driver1.circle.isTapped()) {
            depositor.setState(Depositor.DepositorState.HOME);
            intake.setState(IDLEOPEN);
        } else if (driver1.dpad_up.isTapped()) {
            depositor.setState(Depositor.DepositorState.HIGH);
        } else if (driver1.dpad_left.isTapped()) {
            depositor.setState(Depositor.DepositorState.MID);
        } else if (driver1.dpad_down.isTapped()) {
            depositor.setState(Depositor.DepositorState.LOW);
        } else if (driver1.cross.isTapped()) {
            if(intake.state != FUNNY){
                intake.setState(Intake.IntakeState.IDLECLOSED);
            }else{
                intake.setState(Intake.IntakeState.FUNNYCLOSED);
            }
        } else if (driver1.rightBumper.isTapped()) {
            depositor.setState(Depositor.DepositorState.SCORE);
        } else if (driver1.triangle.isTapped()){
            intake.setState(Intake.IntakeState.FUNNY);
        }else if (driver1.leftBumper.isTapped()){
            intake.setState(SLIDESOUT);
            depositor.setState(Depositor.DepositorState.HOME);
        }


        BaseOpMode.addData("RB1", driver1.rightBumper.isTapped());



    }
}
