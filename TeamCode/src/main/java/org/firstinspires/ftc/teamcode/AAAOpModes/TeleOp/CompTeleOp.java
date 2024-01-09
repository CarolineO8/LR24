package org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;

//@Disabled
@TeleOp(name="AAA COMPETITION TELEOP", group="Iterative Opmode")
public class CompTeleOp extends BaseOpMode {
    //New stuff
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    @Override
    public void externalInit() {
        //On init
        fl = BaseOpMode.hardware.get(DcMotor.class,"fl");
        fr = BaseOpMode.hardware.get(DcMotor.class,"fr");
        br = BaseOpMode.hardware.get(DcMotor.class,"br");
        bl = BaseOpMode.hardware.get(DcMotor.class,"bl");


    }

    @Override
    public void externalLoop() {
        //After start
        double drive = driver1.leftStick.Y();
        double strafe = driver1.leftStick.X();
        double turn = driver1.rightStick.X();
        double speed = 0.5;

        fl.setPower(-(drive - strafe + turn) * speed);
        fr.setPower((drive + strafe - turn) * speed);
        bl.setPower(-(drive + strafe + turn) * speed);
        br.setPower((drive - strafe - turn) * speed);
    }

}