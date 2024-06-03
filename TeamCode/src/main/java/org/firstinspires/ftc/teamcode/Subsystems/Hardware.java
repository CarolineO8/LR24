package org.firstinspires.ftc.teamcode.Subsystems;

//hardware variables followed by hardware objects

//DCMotorEx: https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html?com/qualcomm/robotcore/hardware/DcMotorEx.html

import org.firstinspires.ftc.teamcode.KCP.DriveClasses.MecanumDrive;

public class Hardware{

    public static final String
            clawTilt = "clawPitch";

    public static final String
            guider = "guider";

    public static final String
            depositorClaw = "claw2", turret1 = "clawYaw";

    public static final String
            v4b = "arm1", v4b2 = "arm2";

    public static final String
            verticalEncoder = "verticalEncoder", horizontalEncoder = "horizontalEncoder";

    public static final double verticalEncoderTicksToCM = 0.002414, horizontalEncoderTicksToCM = -0.0018948;

    public static final String
            liftMotor1 = "dr4b", liftMotor2 = verticalEncoder,
            intakeMotor1 = "intakeslides", intakeMotor2 = horizontalEncoder;

    public static final String
            leftFront = "drivefl", rightFront  = "drivefr",
            leftBack = "drivebl", rightBack = "drivebr";

    public static final double[] mecanumWheelPowerVector = new double[]{MecanumDrive.MecanumDriveDash.vecX,MecanumDrive.MecanumDriveDash.vecY};

}
