package org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Utilities.HardwareDevices.Servo;

//@Disabled
@TeleOp(name="AAA TEST", group="Iterative Opmode")
public class TEST extends BaseOpMode {
    Servo wrist;
    Servo leftArm;
    Servo rightArm;
    @Override
    public void externalInit() {
        wrist = new Servo("wrist");
        leftArm = new Servo("arm1",true);
        rightArm = new Servo("arm2");


    }

    @Override
    public void externalLoop() {
if(driver1.circle.isTapped()){
    wrist.setPosition(Dashboard.wristUp);
}
if(driver1.cross.isTapped()){
    leftArm.setPosition(Dashboard.armUp);
    rightArm.setPosition(Dashboard.armUp);
}
    }

}
