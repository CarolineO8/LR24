package org.firstinspires.ftc.teamcode.KCP.Localization;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;

public abstract class Location extends Subsystem {

    protected static double[] location;
    protected static double heading;

    private final ElapsedTime runtime;

    protected final double[] velocity = new double[3];

    public Location(double x, double y){
        super();
        runtime = new ElapsedTime();
        runtime.reset();
        location = new double[]{x, y};
    }

    public static double x(){
        return location[0];
    }

    public static double y(){
        return location[1];
    }

    public static void shiftLocation(double x, double y){
        location[0] -= x;
        location[1] -= y;
    }

    public void setPosition(double x, double y){
        location[0] = x;
        location[1] = y;
    }

    public static double heading(){
        return heading;
    }
    public abstract void localize();

    @Override
    public void update() {
        localize();
        updateVelocity();
    }

    @Override
    public void updateSensors() {

    }

    private int velocityRingBufferCount = 0;
    private final double[][] velocityRingBuffer = new double[][] {{1,0, 0, 999999999},{2,0, 0, 999999999},{3,0, 0, 999999999},{4,0, 0, 999999999},{5,0, 0, 999999999},{6,0, 0, 999999999},{7,0, 0, 999999999},{8,0, 0, 999999999},{9,0, 0, 999999999},{10,0, 0, 999999999} };
    public void updateVelocity(){

        double time = runtime.milliseconds();

        int current = velocityRingBufferCount % 5;
        int last = (velocityRingBufferCount+1) % 5;

        velocityRingBuffer[current][0] = Location.x();
        velocityRingBuffer[current][1] = Location.y();
        velocityRingBuffer[current][2] = Location.heading();
        velocityRingBuffer[current][3] = time;

        double dX = Location.x()-velocityRingBuffer[last][0];
        double dY = Location.y()-velocityRingBuffer[last][1];
        double dH = Location.heading()-velocityRingBuffer[last][2];
        double dT = time-velocityRingBuffer[last][3];

        velocityRingBufferCount++;

        velocity[0] = dX/dT;
        velocity[1] = dY/dT;
        velocity[2] = dH/dT;
    }

    public double[] getVelocity(){
        return velocity;
    }

}
