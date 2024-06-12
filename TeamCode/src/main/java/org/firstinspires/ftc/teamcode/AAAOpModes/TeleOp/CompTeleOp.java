package org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.armDown;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.armUp;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.backClawClosed;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.backClawOpen;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.derivative;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.frontClawClosed;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.frontClawOpen;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.integral;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.proportional;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.wristDown;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.wristUp;


import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.Utilities.HardwareDevices.Gyro;
import org.firstinspires.ftc.teamcode.Utilities.HardwareDevices.Servo;

//@Disabled
@TeleOp(name="AAA COMPETITION TELEOP", group="Iterative Opmode")
public class CompTeleOp extends BaseOpMode {
    //New stuff
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    Servo wrist;
    Servo leftArm;
    Servo rightArm;
    Servo frontClaw;
    Servo backClaw;
    Servo droneLauncher;
    DcMotor slideMotorL;
    DcMotor slideMotorR;
    DcMotor intakeMotor;
    Servo launcher;
    CRServo leftConveyor;
    CRServo counterRoller;
    CRServo rightConveyor;

    int slidePosition = 0;
    Gyro gyro;
    PID pid;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();


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
        wrist = new Servo("wrist");
        launcher = new Servo("launcher");
        leftArm = new Servo("arm1",true);
        rightArm = new Servo("arm2");
        frontClaw = new Servo("frontClaw");
        backClaw = new Servo("backClaw");
        droneLauncher = new Servo("launcher");
        leftConveyor = BaseOpMode.hardware.crservo.get("leftConveyor");
        rightConveyor = BaseOpMode.hardware.crservo.get("rightConveyor");
        counterRoller = BaseOpMode.hardware.crservo.get("counterRoller");
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
        //frontClaw.setPosition(frontClawOpen);
        //backClaw.setPosition(backClawOpen);
        rightArm.setPosition(armDown);
        leftArm.setPosition(armDown);
        wrist.setPosition(wristDown);




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
//        Vector2d rotatedVector = driveVector;



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
            backClaw.setPosition(backClawOpen);
            frontClaw.setPosition(frontClawOpen);
            multTelemetry.addData("door", "should move");
        }

        //LOOK HERE!!!!!!!!!!!!!!!
        if (driver2.rightBumper.isTapped() && up) {
            //transfer
            wrist.setPosition(wristDown);
            //leftArm.setPosition(armDown);
            //rightArm.setPosition(armDown);
            transfer = true;
            time.reset();
        }
        if (transfer) {
            backClaw.setPosition(backClawClosed);
            frontClaw.setPosition(frontClawClosed);
            if (time.seconds() > 0.5) {
                //transfer
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
            rightConveyor.setPower(1);
            leftConveyor.setPower(-1);
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
            rightConveyor.setPower(0);
            leftConveyor.setPower(0);
        }
        if (driver2.triangle.isTapped()) {

            boolean wrist1 = false;
            boolean wrist2 = false;
            //initiate up
          rightArm.setPosition(0.89);
            leftArm.setPosition(0.89);
            wrist.setPosition(0.44);
            wrist1 = true;
            if(wrist1){
                rightArm.setPosition(0.64);
                leftArm.setPosition(0.64);
                wrist.setPosition(0.38);
                wrist2 = true;
            }
            if (wrist2){
                rightArm.setPosition(0.37);
                leftArm.setPosition(0.37);
                wrist.setPosition(0.327);
            }

           /* rightArm.setPosition(0.5);
            leftArm.setPosition(0.5);*/
          //  wrist.setPosition(0.02);


          /*  wrist.setPosition(wristUp);
            rightArm.setPosition(armUp);
            leftArm.setPosition(armUp);*/
            slidesUp = true;
            time.reset();
            up = false;
            down = true;
        }
        if (driver2.share.isTapped()) {
            //depositerDoor.setPosition(0.1);
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
                //depositerDoor.setPosition(0.15);
            }
            if (time.seconds() > 0.5) {
                slidePosition = 10;
                rightArm.setPosition(armDown);
                leftArm.setPosition(armDown);
                wrist.setPosition(wristDown);
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