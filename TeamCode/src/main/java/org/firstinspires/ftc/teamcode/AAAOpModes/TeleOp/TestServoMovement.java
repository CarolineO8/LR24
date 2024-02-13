package org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Servo;

//@Disabled
@TeleOp(name="TEST", group="Iterative Opmode")
public class TestServoMovement extends BaseOpMode {
    //New stuff


    Servo depositer;
    Servo depositerDoor;
    DcMotor slideMotorL;
    DcMotor slideMotorR;
    DcMotor intakeMotor;
    CRServo counterRoller;

    int slidePosition = 0;



    @Override
    public void externalInit() {
        //On init
        depositer = new Servo("depositer");
        depositerDoor = new Servo("depositerDoor");
        counterRoller = BaseOpMode.hardware.crservo.get("roll");
        slideMotorL = BaseOpMode.hardware.get(DcMotor.class,"slideL");
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorL.setTargetPosition(0);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setPower(0.5);
        slideMotorR = BaseOpMode.hardware.get(DcMotor.class,"slideR");
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorR.setTargetPosition(0);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setPower(0.5);
        intakeMotor = BaseOpMode.hardware.get(DcMotor.class,"intake");
    }
    boolean up = true;
    boolean down = false;
    @Override
    public void externalLoop() {
//        slideMotorR.setTargetPosition(slidePosition);
//        slideMotorL.setTargetPosition(slidePosition);
        //After start
        if (driver2.leftBumper.isTapped()) {
            //deposit
            depositerDoor.setPosition(0.05);
        }
        //LOOK HERE!!!!!!!!!!!!!!!
        if (driver2.rightBumper.isTapped()) {
            //transfer
            slidePosition -= 200;
            slideMotorR.setTargetPosition(slidePosition);
            slideMotorL.setTargetPosition(slidePosition);
            depositerDoor.setPosition(0);
            slidePosition += 200;
            slideMotorR.setTargetPosition(slidePosition);
            slideMotorL.setTargetPosition(slidePosition);
        }
        if (driver2.rightTrigger.isToggled()){
            //intake in
            intakeMotor.setPower(-1);
            counterRoller.setPower(-1);
        }
        else if (driver2.leftTrigger.isToggled()) {
            //intake out
            intakeMotor.setPower(1);
            counterRoller.setPower(1);
        }
        else {
            //stop intake
            intakeMotor.setPower(0);
            counterRoller.setPower(0);
        }
        if (driver2.triangle.isTapped() && up) {
            //initiate up
            depositer.setPosition(0.95);
            slidePosition -= 1000;
            slideMotorR.setTargetPosition(slidePosition);
            slideMotorL.setTargetPosition(slidePosition);
            sleep(100);
//            depositer.setPosition(0.55);
            up = false;
            down = true;
        }
        if (driver2.cross.isTapped() && down) {
            //initiate down
            up = true;
            down = false;
            depositerDoor.setPosition(0.15);
            depositer.setPosition(0.95);
            slidePosition += 1000;
            slideMotorR.setTargetPosition(slidePosition);
            slideMotorL.setTargetPosition(slidePosition);
        }
//        if (driver2.dpad_up.isTapped() && slidePosition <= -3000){
//            slidePosition -= 100;
//            slideMotorR.setTargetPosition(slidePosition);
//            slideMotorL.setTargetPosition(slidePosition);
//        }
//        if(driver2.dpad_down.isTapped() && slidePosition >= -1000){
//            slidePosition += 100;
//            slideMotorR.setTargetPosition(slidePosition);
//            slideMotorL.setTargetPosition(slidePosition);
//        }

        telemetry.update();
    }

}