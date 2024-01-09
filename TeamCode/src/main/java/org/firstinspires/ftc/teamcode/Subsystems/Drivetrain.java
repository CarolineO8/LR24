package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Gyro;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Motor;

public class Drivetrain extends Subsystem{

    Motor motorfl;
    Motor motorfr;
    Motor motorbl;
    Motor motorbr;

    double drive;
    double strafe;
    double turn;

    Gamepad gamepad1;
    Gyro gyro;

    @Override
    public void update() {


    }

    @Override
    public void updateSensors() {

    }

    public Drivetrain(){

        gamepad1 = new Gamepad();

        motorfl = new Motor(Hardware.leftFront);
        motorfr = new Motor(Hardware.rightFront);
        motorbl = new Motor(Hardware.leftBack);
        motorbr = new Motor(Hardware.rightBack);

        gyro = new Gyro("imu");



    }

    public void drive(double drive,double strafe,double turn){

       /* drive = gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        turn = gamepad1.left_stick_x;*/

        Vector2d driveVector = new Vector2d(strafe,drive);
        Vector2d rotatedVector = driveVector.rotateBy(Math.toDegrees(-gyro.getHeading()));

        BaseOpMode.addData("Heading", gyro.getHeading());

        drive=rotatedVector.getY();
        strafe=-(rotatedVector.getX());


        motorfl.setPower(-(drive-strafe+turn));
        motorfr.setPower(drive+strafe-turn);
        motorbl.setPower(-(drive+strafe+turn));
        motorbr.setPower(drive-strafe-turn);


    }

}
