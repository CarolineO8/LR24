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
        fl = BaseOpMode.hardware.get(DcMotor.class,"fr");
        fl = BaseOpMode.hardware.get(DcMotor.class,"br");
        fl = BaseOpMode.hardware.get(DcMotor.class,"bl");

    }

    @Override
    public void externalLoop() {

        //After start
    }
}