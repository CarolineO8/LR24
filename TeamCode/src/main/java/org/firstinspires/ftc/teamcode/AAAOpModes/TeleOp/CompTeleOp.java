package org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp;

import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.derivative;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.integral;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.proportional;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
    DcMotor intakeMotor;
    DcMotor slideMotorL;
    DcMotor slideMotorR;
    CRServo counterRoller;
    Servo depositerDoor;
    Servo depositer;
    Gyro gyro;
    PID pid;


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
        slideMotorL = BaseOpMode.hardware.get(DcMotor.class,"slideMotorL");
        gyro = new Gyro("imuA");
        pid = new PID(proportional,integral,derivative);
        counterRoller = BaseOpMode.hardware.get(CRServo.class, "counterRoller");


    }

    @Override
    public void externalLoop() {
        //After start
        double drive = -driver1.leftStick.Y();
        double strafe = -driver1.leftStick.X();
        double turn = driver1.rightStick.X();
        double speed = 0.7;

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

        if (driver1.rightBumper.isPressed()) {
            speed = 0.2;
        }
        else if (driver1.rightTrigger.isPressed()) {
            speed = 0.4;
        }

        fl.setPower(-(drive - strafe + turn) * speed);
        fr.setPower((drive + strafe - turn) * speed);
//        bl.setPower(-(drive + strafe + turn) * speed);
//        br.setPower((drive - strafe - turn) * speed);
        bl.setPower((-(drive + strafe + turn) * speed) * 11/12);
        br.setPower(((drive - strafe - turn) * speed) * 11/12);
        if (driver2.rightTrigger.isPressed()) {
            //
            intakeMotor.setPower(0.8);
            counterRoller.set(0.8);
        }
        if (driver2.leftTrigger.isPressed()) {
            //intake out
            intakeMotor.setPower(-0.8);
            counterRoller.set(-0.8);

        }
        if (driver2.rightBumper.isTapped()) {
            slideMotorL.setTargetPosition(100);
            slideMotorR.setTargetPosition(100);
            depositerDoor.setPosition(0);

        }
        if (driver2.leftBumper.isTapped()) {

        }
        if (driver2.dpad_down.isTapped()) {

        }
            slideMotorL.setPower(driver2.leftStick.Y());
            slideMotorR.setPower(driver2.leftStick.X());
        if (driver2.triangle.isTapped()) {

        }


        BaseOpMode.addData("heading", gyro.getHeading());
    }

}