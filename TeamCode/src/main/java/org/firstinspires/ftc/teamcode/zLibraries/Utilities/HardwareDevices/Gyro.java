package org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices;

import static com.arcrobotics.ftclib.kotlin.extensions.geometry.Vector2dExtKt.getAngle;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;

import java.util.ArrayList;

public class Gyro {

    private final BNO055IMU controlHubIMU;

    public static ArrayList<Gyro> gyros = new ArrayList<>();
    private double wrappedHeading = 0;
    private double rawHeading = 0;
    private double offset = 0;
    double previousTime = 0;
    double previousChange = 0;


    public Gyro(String name){
        gyros.add(this);
        controlHubIMU = BaseOpMode.hardware.get(BNO055IMU.class, name);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        controlHubIMU.initialize(parameters);



        setCurrentHeading(0);


    }


    public double getHeading(){
        return wrappedHeading;
    }


    //maybe make total heading
    public double getRawHeading(){
        return rawHeading;
    }

    public double getUnwrappedHeading(){
        return rawHeading - offset;
    }

    public void resetHeading(){
        offset += rawHeading;
    }



    //In radians
    public void setCurrentHeading(double heading){
        offset = -heading;
    }

    public static void updateAngles(){
        for(Gyro g : gyros){
            g.update();
        }
    }

    public double getVeryRawHeading(){
        return 0;
    }

    public double getRateOfChange(){
        double change = getHeading();
        double deltaTime = (System.currentTimeMillis() - previousTime) / 1000.0;
        double deltaChange = change - previousChange;
        double rateOfChange = deltaChange/deltaTime;
        previousTime = System.currentTimeMillis();
        previousChange = change;
        return Math.abs(rateOfChange);
    }

    public static void resetGyroList(){
        gyros.clear();
    }

    private void update(){

        //TODO revHub orientation might matter
        double rawYaw;
        Orientation angles = controlHubIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        rawYaw = angles.firstAngle;

        rawHeading = rawYaw;

        wrappedHeading = wrapAngle(rawYaw) - offset;
//        BaseOpMode.addData("offset", offset);

    }

    private double wrapAngle(double angle){
//        angle += 2 * Math.PI;

        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }

        while (angle < -Math.PI){
            angle += 2 * Math.PI;
        }



        return angle;
    }

}
