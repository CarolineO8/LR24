package org.firstinspires.ftc.teamcode.KCP.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;

@Autonomous(name="Odometer Test", group="Testing")
public class OdometryTest extends BaseOpMode {

    public long lastLoop;

    TwoWheelOdometry odo;

    @Override
    public void externalInit() {
        odo = new TwoWheelOdometry(0,0);

        BaseOpMode.addData("status","initialized");
    }

    @Override
    public void externalStart() {
        lastLoop = System.nanoTime();
    }

    @Override
    public void externalLoop() {
        odo.localize();
        double[] Raw = odo.getRawValues();
//            double dTime = System.nanoTime() - lastLoop;
//            lastLoop = System.nanoTime();
//            double hz = 1000000000.0 / (dTime);
//            WolfpackOpMode.addData("Heartz <3", hz);
        BaseOpMode.addData("X", Location.x());
        BaseOpMode.addData("Y", Location.y());
        BaseOpMode.addData("Y", Location.heading());
//
        BaseOpMode.addData("\t", " ");
        BaseOpMode.addData("Veritical Encoder", Raw[0]);
        BaseOpMode.addData("Horizontal Encoder", Raw[1]);
    }
}
