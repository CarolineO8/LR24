package org.firstinspires.ftc.teamcode.KCP;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous.Paths;
import org.firstinspires.ftc.teamcode.KCP.Localization.Location;
import org.firstinspires.ftc.teamcode.KCP.Localization.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.KCP.DriveClasses.AbstractClasses.DriveTrain;
import org.firstinspires.ftc.teamcode.KCP.DriveClasses.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public class Movement extends Subsystem {
    /**
     * In this case odometry, but any extension of the localization class can be passed in
     */
    private final Location odo;
    /**
     * Any type of drivebase that extends the drivetrain class
     */
    public final DriveTrain drive;
    private final ElapsedTime runtime;

    @Config
    public static class MovementDash {
        public static double headingVelocityThreshold = 0.0002, movementVelocityThreshold = 0.02/3, movementFastVelocityThreshold = .3;
    }

    /**
     * Delcare drivebase and localization method
     * @param startX
     * @param startY
     */
    public Movement(double startX, double startY){
        drive = new MecanumDrive();

        drive.lockDrive();
        drive.stopDrive();

        odo = new TwoWheelOdometry(startX, startY);

        runtime = new ElapsedTime();
        runtime.reset();
    }

    /**
     * Needs to be called every loop
     */
    public void update(){

    }

    @Override
    public void updateSensors() {
        
    }

    boolean velocityWasHigh = false;
    int stoppedCount = 0;

    /**
     * Function needs to be continuously called to continue to follow path
     * @param path - Enum with constructed path to follow
     * @param power - Maximum driving power
     * @param heading - Target heading
     * @param tThreshold - Stop at end false (default): the t value at which the path will stop being followed
     *                   - Stop at end true: the minimum t value the robot will begin to slow down and target endpoint
     * @param stopAtEnd - Whether the robot stops when the path is complete
     * @param stoppedVelocity - Velocity to determine when the robot has settled and the path is complete
     * @return - if path is being followed (ie not complete)
     */
    public boolean followPath(Paths.Path path, double power, double heading, double tThreshold, boolean stopAtEnd, double stoppedVelocity){
        //Get velocities
        double[] v = odo.getVelocity();
        double headingVelocity = v[2];
        double[] velocity = new double[]{v[0], v[1]};
        double velocityMagnitude = Math.hypot(velocity[0], velocity[1]);

        //Predicted stopping point for decing when and how to slow down if stop at end
        double predictedStoppingScalar = velocityMagnitude/drive.getDecelerationConstant();
        double[] predictedStoppingVector = new double[]{velocity[0] * predictedStoppingScalar, velocity[1] * predictedStoppingScalar};
        double predictedStoppingVectorMagnitude = Math.hypot(predictedStoppingVector[0], predictedStoppingVector[1]);

        //Counter to cancel movement if stopped for too long
        if(((Math.abs(headingVelocity) <= MovementDash.headingVelocityThreshold && velocityMagnitude <= stoppedVelocity))){
            stoppedCount++;
        }
//TODO
//        System.out.println("Stopped Count : " + stoppedCount);
        //Robot determined to be in unrecoverable position (ie stuck)
        if (stoppedCount > 20) {
            //cancel path
            drive.stopDrive();
            path.resetPath();
            velocityWasHigh = false;
            stoppedCount = 0;
            return false;
        }

        //If time to target end point instead of following path (stop at end true)
        if(path.targeting || (stopAtEnd && ((path.t >= tThreshold && (predictedStoppingVectorMagnitude > path.distanceToTarget())) || path.t>=1))){
            path.targeting = true;
//TODO
//            System.out.println("Heading Velocity : " + headingVelocity + "\tVelocity : " + velocityMagnitude);

            //if robot is settled
            if((Math.abs(headingVelocity) <= MovementDash.headingVelocityThreshold && velocityMagnitude <= stoppedVelocity)){
                stoppedCount = 0;
                drive.stopDrive();
                path.resetPath();
                velocityWasHigh = false;
                return false;
            }

            //Target endpoint based on current predicted stopping position
            if(Math.hypot(path.endPoint[0] - Location.x(), path.endPoint[1] - Location.y()) < drive.getTargetingThreshold()){
                holdPosition(path.endPoint[0], path.endPoint[1], heading);
            }else{
                holdPosition(path.endPoint[0]-predictedStoppingVector[0], path.endPoint[1]-predictedStoppingVector[1], heading);
            }

        }else{
            //Path following section of movement

            //if time to cancel path
            if(!stopAtEnd && path.t > tThreshold){
                path.resetPath();
                velocityWasHigh = false;
                stoppedCount = 0;
                return false;
            }

            //distance of robot to last closest point
            double dist = Math.pow(path.x - Location.x(), 2) + Math.pow(Location.y()  - path.y, 2);

            //incrementing t to find new closest t
            double newT = path.t + path.tIncrement;
            double newX = path.getX(newT);
            double newY = path.getY(newT);
            double newDist = Math.pow(newX - Location.x(), 2) + Math.pow(Location.y()  - newY, 2);

            //while the distance is decreasing (point is moving closer to the robot) keep incrementing t
            while(dist > newDist){
                path.t = newT;
                path.x = newX;
                path.y = newY;

                dist = newDist;

                newT += path.tIncrement;

                newX = path.getX(newT);
                newY = path.getY(newT);
                newDist = Math.pow(newX - Location.x(), 2) + Math.pow(Location.y()  - newY, 2);

            }
            //newT is effectively seeking point - t is the closest to the robot and newT is one increment further - then used to draw a secant line to follow

            path.updatePathDerivatives();

            double headingCorrection = heading - Location.heading();

            //finds angle between line newT-t and t-location, the sign of the value determines which side of the path the robot is on, then used for correcting PID
            double pathVectorAngle = Math.atan2(path.dy, path.dx);
            double pathDisplacementAngle = (new Rotation2d(Math.atan2(path.y-Location.y(), path.x-Location.x()) - pathVectorAngle)).getRadians();

            //correcting PID, uses distance from the path the apply a power perpendicular to the secant line
            double pathDisplacement = Math.sqrt(dist) * Math.signum(pathDisplacementAngle);
            if(path.curvy){
                //radius of curvature of the path
                double arcRad = Math.pow(Math.hypot(path.dx, path.dy), 3)
                        / (path.dx * path.d2y - path.dy * path.d2x);
                //drive
                drive.followArc(Math.atan2(path.dy, path.dx), power, headingCorrection, pathDisplacement, arcRad, velocity);
            }else{
                //drive
                drive.drive(Math.atan2(path.dy, path.dx), power, headingCorrection, pathDisplacement);
            }

        }

        return true;
    }

    /**
     * Robot should always be trying to hold a position in autonomous
     * @param x - x position to hold
     * @param y - y position to hold
     * @param h - heading to hold
     * @param hF - heading factor for determining when stopped
     * @param mF - movement factor for determining when stopped
     * @return - if robot is settled
     */
    public boolean holdPosition(double x, double y, double h, double hF, double mF){
        double[] target = new double[]{x - Location.x(), y - Location.y()};
        double targetAngle = Math.atan2(target[1], target[0]);

        drive.drive(targetAngle, drive.getHoldPositionPower(Math.hypot(target[0], target[1])) * drive.getDirectionalPowerScalar(targetAngle - Location.heading(), true), h - Location.heading());

        double[] v = odo.getVelocity();
        double headingVelocity = v[2];
        double velocityMagnitude = Math.hypot(v[0], v[1]);
        return (Math.abs(headingVelocity) <= MovementDash.headingVelocityThreshold * hF && velocityMagnitude <= MovementDash.movementVelocityThreshold * mF);
    }

    public boolean holdPosition(double x, double y, double h) {
        return holdPosition(x,y,h,1, 2);
    }

    public boolean followPath(Paths.Path path, double power, double heading, double tThreshold, boolean stopAtEnd) {
        return followPath(path, power, heading, tThreshold, stopAtEnd, MovementDash.movementVelocityThreshold);
    }

    public boolean followPath(Paths.Path path, double power, double heading, double tThreshold) {
        return followPath(path, power, heading, tThreshold, false, MovementDash.movementVelocityThreshold);
    }

    public boolean followPath(Paths.Path path, double power, double heading) {
        return followPath(path, power, heading, .95, false, MovementDash.movementVelocityThreshold);
    }

    public boolean followPath(Paths.Path path, double power) {
        return followPath(path, power, 0, .95, false, MovementDash.movementVelocityThreshold);
    }

    public void shiftLocation(double x, double y){
        Location.shiftLocation(x,y);
    }

    public void stopDrive(){
        drive.stopDrive();
    }

}
