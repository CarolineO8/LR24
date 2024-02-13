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


    @Override
    public void externalInit() {
        //On init
        depositer = new Servo("depositer");
        depositerDoor = new Servo("depositerDoor");
        slideMotorL = BaseOpMode.hardware.get(DcMotor.class,"slideL");
        slideMotorR = BaseOpMode.hardware.get(DcMotor.class,"slideR");


    }

    @Override
    public void externalLoop() {
        //After start
        if (driver2.leftBumper.isTapped()) {
            depositerDoor.setPosition(0.05);
        }
        if (driver2.leftTrigger.isTapped()) {
            depositerDoor.setPosition(0);

        }
        if (driver2.rightBumper.isTapped()) {
            depositer.setPosition(0.55);
        }
        if (driver2.rightTrigger.isTapped()) {
            depositer.setPosition(0.95);
        }
        if (driver2.dpad_up.isToggled()) {
            slideMotorL.setPower(-0.7);
            slideMotorR.setPower(-0.7);
        }
//        if (driver2.dpad_down.isPressed()) {
//            slideMotorL.setPower(0.7);
//            slideMotorR.setPower(0.7);
//        }
        telemetry.update();
    }

}