package org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Gyro;

//@Disabled
@TeleOp(name="AAA COMPETITION TELEOP", group="Iterative Opmode")
public class CompTeleOp extends BaseOpMode {
    //New stuff
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
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
        gyro = new Gyro("imu");
        pid = new PID(0,0,0);


    }

    @Override
    public void externalLoop() {
        //After start
        double drive = driver1.leftStick.Y();
        double strafe = driver1.leftStick.X();
        double turn = driver1.rightStick.X();
        double speed = 0.5;

        Vector2d driveVector = new Vector2d(strafe, drive);
        Vector2d rotatedVector = driveVector.rotateBy(Math.toDegrees(270-gyro.getHeading()));


        drive = rotatedVector.getY();
        strafe = -rotatedVector.getX();


        //TODO PID STUFF
        double currentRateOfChange = gyro.getRateOfChange();
        if (turn != 0){ pid_on = false;}
        else if (currentRateOfChange <= 120) pid_on = true;

        if (pid_on && !pid_on_last_cycle) {
            setPoint = gyro.getHeading();
        }else if (pid_on){
            turn = pid.getCorrection(gyro.getHeading(), setPoint);
        }
        pid.setConstants(0,0,0);
        pid_on_last_cycle = pid_on;


        fl.setPower(-(drive - strafe + turn) * speed);
        fr.setPower((drive + strafe - turn) * speed);
        bl.setPower(-(drive + strafe + turn) * speed);
        br.setPower((drive - strafe - turn) * speed);
    }

}