package org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.derivative;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.integral;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.proportional;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Gyro;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Servo;

//@Disabled
@TeleOp(name="AAA COMPETITION TELEOP", group="Iterative Opmode")
public class CompTeleOp extends BaseOpMode {
    //New stuff
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    Servo depositer;
    Servo depositerDoor;
    DcMotor slideMotorL;
    DcMotor slideMotorR;
    DcMotor intakeMotor;
    Servo launcher;
    CRServo counterRoller;

    int slidePosition = 0;
    Gyro gyro;
    PID pid;

    ElapsedTime time = new ElapsedTime();


    boolean pid_on = false;
    boolean pid_on_last_cycle = false;
    double setPoint = 0;

    @Override
    public void externalInit() {
        //On init
        fl = BaseOpMode.hardware.get(DcMotor.class,"fl");
        fr = BaseOpMode.hardware.get(DcMotor.class,"fr");
        br = BaseOpMode.hardware.get(DcMotor.class,"br");
        bl = BaseOpMode.hardware.get(DcMotor.class,"bl");
        depositer = new Servo("depositer");
        depositerDoor = new Servo("depositerDoor");
        launcher = new Servo("launcher");
        counterRoller = BaseOpMode.hardware.crservo.get("roll");
        slideMotorL = BaseOpMode.hardware.get(DcMotor.class,"slideL");
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorL.setTargetPosition(0);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setPower(0.8);
        slideMotorR = BaseOpMode.hardware.get(DcMotor.class,"slideR");
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorR.setTargetPosition(0);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setPower(0.8);
        intakeMotor = BaseOpMode.hardware.get(DcMotor.class,"intake");
        gyro = new Gyro("imuA");
        pid = new PID(proportional,integral,derivative);



    }
    boolean up = true;
    boolean down = false;
    boolean transfer = false;
    boolean slidesUp = false;
    double speed;

    boolean slidesDown;
    @Override
    public void externalLoop() {
        //After start
        double drive = driver1.leftStick.Y();
        double strafe = driver1.leftStick.X();
        double turn = driver1.rightStick.X();

        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotateBy(Math.toDegrees(270-gyro.getHeading()));


        drive = rotatedVector.getY();
        strafe = -rotatedVector.getX();


        //TODO PID STUFF
        //assigns the current rate of change variable to the speed of rotation
        double currentRateOfChange = gyro.getRateOfChange();
        //turns PID off when turning
        if (turn != 0){ pid_on = false;}
        //when the speed of rotation is less then 120 than turn PID on.
        else if (currentRateOfChange <= 120) pid_on = true;
        //if PID is on and not on last cycle then
        if (pid_on && !pid_on_last_cycle) {
            setPoint = gyro.getHeading();
        }
        //if PID is on and on last cycle then it corrects it.
        else if (pid_on){
            turn = pid.getCorrectionHeading(gyro.getHeading(), setPoint);
        }
        //sets gain
        pid.setConstants(proportional,integral,derivative);
        pid_on_last_cycle = pid_on;

        if (driver1.rightTrigger.isPressed() || slidePosition <= -50) {
            speed = 0.4;
        }
        else if (driver1.rightBumper.isPressed()) {
            speed = 0.2;
        }
        else {
            speed = 0.7;
        }

        fl.setPower(-(drive - strafe + turn) * speed);
        fr.setPower((drive + strafe - turn) * speed);
//        bl.setPower(-(drive + strafe + turn) * speed);
//        br.setPower((drive - strafe - turn) * speed);
        bl.setPower((-(drive + strafe + turn) * speed) * 11/12);
        br.setPower(((drive - strafe - turn) * speed) * 11/12);
//        slideMotorR.setTargetPosition(slidePosition);
//        slideMotorL.setTargetPosition(slidePosition);
        //After start
        slideMotorR.setTargetPosition(slidePosition);
        slideMotorL.setTargetPosition(slidePosition);
        //After start
        if (driver2.leftBumper.isTapped()) {
            //deposit
            depositerDoor.setPosition(0.05);
            multTelemetry.addData("door", "should move");
        }

        //LOOK HERE!!!!!!!!!!!!!!!
        if (driver2.rightBumper.isTapped() && up) {
            //transfer
            transfer = true;
            depositer.setPosition(1);
            time.reset();
        }
        if (transfer) {
            if (time.seconds() > 0.5) {
                depositerDoor.setPosition(0);
                transfer = false;
            }
        }
        if(driver1.triangle.isTapped()){
            gyro.resetHeading();
        }
        if (driver2.rightTrigger.isPressed()){
            //intake in
            intakeMotor.setPower(-1);
            counterRoller.setPower(-1);
        }
        else if (driver2.leftTrigger.isPressed()) {
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
            depositer.setPosition(0.85);
            slidesUp = true;
            time.reset();
            up = false;
            down = true;
        }
        if (driver2.touchpad.isTapped()) {
            depositer.setPosition(0.85);
            depositerDoor.setPosition(0.1);
        }
        if (slidesUp) {
            if (time.seconds() > 0) {
                slidePosition = -1000;
            }
            if (time.seconds() > 1) {
                depositerDoor.setPosition(0);
                depositer.setPosition(0.55);
                slidesUp = false;
            }
        }
        if (driver2.cross.isTapped() && down) {
            //initiate down
            up = true;
            down = false;
            slidesDown = true;
            time.reset();
        }
        if (slidesDown) {
            if (time.seconds() > 0) {
                depositerDoor.setPosition(0.15);
                depositer.setPosition(0.85);
            }
            if (time.seconds() > 0.5) {
                slidePosition = 10;
                slidesDown = false;
            }
        }
        if (driver2.dpad_up.isTapped() && slidePosition >= -1000){
            slidePosition -= 100;
        }
        if (driver2.dpad_down.isTapped() && slidePosition <= 0){
            slidePosition += 100;
        }
        if (driver2.square.isTapped()) {
            launcher.setPosition(0.4);
        }

        telemetry.update();


        BaseOpMode.addData("heading", gyro.getHeading());
    }

}