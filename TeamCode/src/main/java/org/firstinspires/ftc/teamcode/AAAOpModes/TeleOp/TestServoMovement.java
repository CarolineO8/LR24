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
@TeleOp(name="TEST", group="Iterative Opmode")
public class TestServoMovement extends BaseOpMode {
    //New stuff


    Servo depositer;
    Servo depositerDoor;
    DcMotor slideMotorL;
    DcMotor slideMotorR;
    DcMotor intakeMotor;



    @Override
    public void externalInit() {
        //On init
        depositer = new Servo("depositer");
        depositerDoor = new Servo("depositerDoor");
        slideMotorL = BaseOpMode.hardware.get(DcMotor.class,"slideL");
        slideMotorR = BaseOpMode.hardware.get(DcMotor.class,"slideR");
        intakeMotor = BaseOpMode.hardware.get(DcMotor.class,"intakeMotor");


    }

    @Override
    public void externalLoop() {
        //After start
        double slides = -driver2.leftStick.Y();
        if (driver2.leftBumper.isTapped()) {
            depositerDoor.setPosition(0.05);
        }
        if (driver2.leftTrigger.isTapped()) {
            depositerDoor.setPosition(0);
        }
        if (driver2.dpad_down.isTapped()) {
            depositerDoor.setPosition(0.1);
        }
        if (driver2.rightBumper.isTapped()) {
            depositer.setPosition(0.55);
        }
        if (driver2.rightTrigger.isTapped()) {
            depositer.setPosition(0.95);
        }
        while (driver2.cross.isPressed()) {
            intakeMotor.setPower(1);
        }
        while (driver2.circle.isPressed()) {
            intakeMotor.setPower(-1);
        }
        slideMotorL.setPower((slides) * 0.7);
        slideMotorR.setPower((slides) * 0.7);
        intakeMotor.setPower(0);
        telemetry.update();
    }

}