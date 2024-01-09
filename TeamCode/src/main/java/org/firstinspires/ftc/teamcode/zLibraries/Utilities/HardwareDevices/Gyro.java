package org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;

import java.util.ArrayList;

public class Gyro {

    private BNO055IMU controlHubIMU;

    public static ArrayList<Gyro> gyros = new ArrayList<>();
    private double wrappedHeading = 0;
    private double rawHeading = 0;
    private double offset = 0;

    public Gyro(String name){
        gyros.add(this);
        controlHubIMU = imuConstructor(name);
    }

    public double getHeading(){
        return wrappedHeading;
    }

    //maybe make total heading
    public double getRawHeading(){
        return rawHeading;
    }

    public void resetHeading(){
        offset += rawHeading;
    }

    public void setCurrentHeading(double heading){
        resetHeading();
        offset -= heading;
    }

    public static void updateAngles(){
        for(Gyro g : gyros){
            g.update();
        }
    }

    public static void resetGyroList(){
        gyros.clear();
    }

    private void update(){
        Orientation angles = controlHubIMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //TODO revHub orientation might matter
        double rawYaw = angles.firstAngle - offset;
//        double rawPitch = angles.secondAngle;
//        double rawRoll = angles.thirdAngle;

        rawHeading = rawYaw;
        wrappedHeading = wrapAngle(rawYaw);

    }

    private double wrapAngle(double angle){
        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }

        while (angle < -Math.PI){
            angle += 2 * Math.PI;
        }

        return angle;
    }

    private BNO055IMU imuConstructor(String deviceName){
        BNO055IMU imu;
        imu = BaseOpMode.hardware.get(BNO055IMU.class, deviceName);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);
        return imu;
    }
}
