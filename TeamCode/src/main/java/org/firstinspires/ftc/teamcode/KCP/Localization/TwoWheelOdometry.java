package org.firstinspires.ftc.teamcode.KCP.Localization;

import org.firstinspires.ftc.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Gyro;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.MotorEncoder;

public class TwoWheelOdometry extends Location{

    //et variables and measure precision of instruments
    private double imuOffset = 0;
    private int imuNumber = 0;

    //location variables to be read by other classes
    double h = 0;

    //what the imu reading should be rounded to for maximum accuracy
    double IMUMaximumPrecision = 0.01;
    double IMUMaxNum;

    //defining the center of the robot to be about the center of the vertical encoder
    double horizontalOffset = 13; //vertical distance from horizontal encoder to center of robot
    double verticalOffset = 14.5; //

    //last vertical and horizontal encoder readings
    double hPrevDist, vPrevDist;

    //new readings
    double newVertical, newHorizontal, newHeading;
    //change in readings / calculated change
    double dVertical, dHorizontal, dHeading, dX, dY;
    final MotorEncoder verticalEncoder;
    final MotorEncoder horizontalEncoder;
    public final Gyro gyro;

    //intialize odometry
    public TwoWheelOdometry(double startX, double startY){
        super(startX, startY);
        IMUMaxNum = 1/IMUMaximumPrecision;

        verticalEncoder = new MotorEncoder(Hardware.verticalEncoder, Hardware.verticalEncoderTicksToCM);
        horizontalEncoder = new MotorEncoder(Hardware.horizontalEncoder, Hardware.horizontalEncoderTicksToCM);

        gyro = new Gyro( "imu");

        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)
        verticalEncoder.resetEncoder();
        horizontalEncoder.resetEncoder();
        vPrevDist = verticalEncoder.getPosition();
        hPrevDist = horizontalEncoder.getPosition();
//
    }

    public void localize(){

        //get new reads on sensors (heading, Vertical encoder, and horizontal encoder)

        newVertical = verticalEncoder.getPosition();
        newHorizontal = horizontalEncoder.getPosition();
        newHeading = gyro.getHeading();

        //get change in heading, Vertical encoder, and horizontal encoder
        dVertical = newVertical - vPrevDist;
        dHorizontal = newHorizontal - hPrevDist;
        dHeading = newHeading - h;

        while (dHeading < -Math.PI) { // For example 355 to 5 degrees
            dHeading += 2 * Math.PI;
        }
        while (dHeading > Math.PI) { // For example 5 to 355 degrees // IDT NECESSARY
            dHeading -= 2 * Math.PI;
        }

        //Math: https://www.desmos.com/calculator/sfpde8dhcw - incorrect
        //needs more images to be properly explained

        //catch the divide by 0
        //TODO use line based when delta heading in range
        if(dHeading == 0) {
            dX = dHorizontal;
            dY = dVertical;
        }else {


            //normal odometry

//            double arcRad = (dVertical - (verticalOffset * dHeading)) / dHeading;
//            ///  \/ highly debated
//            double number = arcRad + dHorizontal - (horizontalOffset * dHeading);
//
//            dX = fastCos(dHeading) * (number) - arcRad;
//            dY = fastSin(dHeading) * (number);
//
//            dX = dHorizontal * fastCos(dHeading) + dVertical * fastSin(dHeading);
//            dY = dHorizontal * fastSin(dHeading) + dVertical * fastCos(dHeading);

            //
            double arcRad1 = (dVertical - (verticalOffset * dHeading)) / dHeading;
            //
            double arcRad2 = (dHorizontal - (horizontalOffset * dHeading)) / dHeading;


            dY = Math.sin(dHeading) * arcRad1 - Math.cos(dHeading) * arcRad2 + arcRad2;
            dX = Math.sin(dHeading) * arcRad2 + Math.cos(dHeading) * arcRad1 - arcRad1;
        }


        //rotating to absolute coordinates vs robot relative calculated above
        location[0] += Math.cos(h) * dX - Math.sin(h) * dY;
        location[1] += Math.sin(h) * dX + Math.cos(h) * dY;
        Location.heading = newHeading;

        //update reference values to current position
        vPrevDist = newVertical;
        hPrevDist = newHorizontal;
        h = newHeading;
    }


    public double[] getRawValues(){
        return new double[]{verticalEncoder.getPosition(), horizontalEncoder.getPosition(), newHeading};
    }


}