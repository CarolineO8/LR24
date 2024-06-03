package org.firstinspires.ftc.teamcode.KCP.DriveClasses;

import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.KCP.DriveClasses.AbstractClasses.StaticDrive.StaticDriveTrain;
import org.firstinspires.ftc.teamcode.KCP.DriveClasses.AbstractClasses.StaticDrive.StaticDriveWheel;
import org.firstinspires.ftc.teamcode.Subsystems.Hardware;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Motor;

public class MecanumDrive extends StaticDriveTrain {

    PIDController headingPID, perpendicularPowerPID, holdPositionPID;

    @Config
    public static class MecanumDriveDash{
        public static double FcConstant = 3200, DecelerationConstant = 0.0007, TargetingThreshold = 0;

        
        public static double headingP = 0.6, headingI = 0, headingD = 0;
        public static double ppP = 0.03, ppI = 0, ppD = 0;
        public static double holdP = 0.07, holdI = 0, holdD = 0;
        public static double driveMinimumPower = .3, headingMaxPower = .6;
        public static double vecX = .11, vecY = .16;

        public static double maxMovePIDPower = .7;
    }

    public MecanumDrive(boolean teleop){
        super(4);
    }

    public MecanumDrive() {
        super(4);

        headingPID = new PIDController(0,0,0);
        perpendicularPowerPID = new PIDController(0,0,0);
        holdPositionPID = new PIDController(0,0,0);

        double[] leftHandWheelVector = Hardware.mecanumWheelPowerVector;
        double[] rightHandWheelVector = new double[]{-leftHandWheelVector[0], leftHandWheelVector[1]};

        StaticDriveWheel rightFront = new StaticDriveWheel(Hardware.rightFront, rightHandWheelVector, 1, false);
        StaticDriveWheel rightBack = new StaticDriveWheel(Hardware.rightBack, leftHandWheelVector, 1, false);
        StaticDriveWheel leftFront = new StaticDriveWheel(Hardware.leftFront, leftHandWheelVector,  -1, true);
        StaticDriveWheel leftBack = new StaticDriveWheel(Hardware.leftBack, rightHandWheelVector, -1, true);
        Motor.printMotorList();
        driveWheels = new StaticDriveWheel[]{rightFront, rightBack, leftFront, leftBack};
        setDriveWheels(driveWheels);
        Motor.printMotorList();

        lockDrive();
    }

    protected void setWheelPowers(double targetAngle, double power, double headingPower){
        double rightHandWheelAngle = driveWheels[0].getAngle() + (-targetAngle);
        double leftHandWheelAngle = driveWheels[2].getAngle() + (-targetAngle);

        double rightHandWheelX = signum(Math.cos(rightHandWheelAngle));
        double leftHandWheelX = signum(Math.cos(leftHandWheelAngle));

        double rightHandWheelVectorY = Math.sin(rightHandWheelAngle);
        double leftHandWheelVectorY = Math.sin(leftHandWheelAngle);

        double leftHandWheelPower = power;
        double rightHandWheelPower = power;

        if (rightHandWheelX == 0) {
            //Make vector point right
            leftHandWheelPower *= leftHandWheelX;
            //Take minus y component of left vector pointing right and multiply it by the vertical right wheel vector
            rightHandWheelPower *= signum(rightHandWheelVectorY) * -leftHandWheelVectorY * leftHandWheelX;

        }else if(leftHandWheelX == 0){
            rightHandWheelPower *= rightHandWheelX;
            leftHandWheelPower *= signum(leftHandWheelVectorY) * -rightHandWheelVectorY * rightHandWheelX;

        }else{

            if(abs(rightHandWheelVectorY) > abs(leftHandWheelVectorY)){
                //make sure right hand vector is pointing right and scale so has the same vertical component as left hand
                rightHandWheelPower *= rightHandWheelX * Math.abs(leftHandWheelVectorY / rightHandWheelVectorY);
                //make sure left hand vector is facing right
                leftHandWheelPower *= leftHandWheelX;

            } else {
                rightHandWheelPower *= rightHandWheelX;
                leftHandWheelPower *= leftHandWheelX * Math.abs(rightHandWheelVectorY / leftHandWheelVectorY);

            }
        }

        driveWheels[0].setPower(rightHandWheelPower - headingPower);
        driveWheels[3].setPower(rightHandWheelPower + headingPower);
        driveWheels[1].setPower(leftHandWheelPower - headingPower);
        driveWheels[2].setPower(leftHandWheelPower + headingPower);
    }

    /**
     * Scalar to convert given power to be equivalent in all drive directions based on favored
     * Doesn't work great on diagonals
     * @param targetAngle - target angle of movement
     * @param forwardOrSideways - true if wanting power scalar in direction driving, false if used for perpendicular power
     * @return - scalar for good drive
     */
    public double getDirectionalPowerScalar(double targetAngle, boolean forwardOrSideways){
        double rightHandWheelAngle = driveWheels[0].getAngle() + (-targetAngle);
        double leftHandWheelAngle = driveWheels[2].getAngle() + (-targetAngle);

        if(!forwardOrSideways){
            rightHandWheelAngle += Math.PI * .5;
            leftHandWheelAngle += Math.PI * .5;
        }

        return (abs(Math.sin(leftHandWheelAngle)) + abs(Math.sin(rightHandWheelAngle))) * 2 / largestPowerVector;
    }

    @Override
    public double getHoldPositionPower(double error) {
        //PID
        holdPositionPID.setPID(MecanumDriveDash.holdP, MecanumDriveDash.holdI, MecanumDriveDash.holdD);
        return Range.clip(holdPositionPID.calculate(error), -MecanumDriveDash.maxMovePIDPower, MecanumDriveDash.maxMovePIDPower);
    }

    @Override
    public double getHeadingPower(double headingError){
        //PID
        headingPID.setPID(MecanumDriveDash.headingP, MecanumDriveDash.headingI, MecanumDriveDash.headingD);
        return Range.clip(headingPID.calculate(headingError), -MecanumDriveDash.headingMaxPower, MecanumDriveDash.headingMaxPower);
    }

    @Override
    public double getPerpendicularPower(double perpendicularError){
        //PID
        perpendicularPowerPID.setPID(MecanumDriveDash.ppP, MecanumDriveDash.ppI, MecanumDriveDash.ppD);
        return perpendicularPowerPID.calculate(perpendicularError);
    }

    @Override
    public double getDecelerationConstant() {
        return MecanumDriveDash.DecelerationConstant;
    }

    @Override
    public double getTargetingThreshold() {
        //10
        return MecanumDriveDash.TargetingThreshold;
    }

    @Override
    public double getCentripetalForceConstant() {
        return MecanumDriveDash.FcConstant;
    }

    @Override
    public double getDriveMinimumMovementPower() {
        return MecanumDriveDash.driveMinimumPower;
    }

}
