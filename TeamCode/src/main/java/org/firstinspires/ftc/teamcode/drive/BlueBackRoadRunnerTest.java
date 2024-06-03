//package org.firstinspires.ftc.teamcode.drive;
//
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//
//@Autonomous(name="Blue back Auto", group="Linear Opmode")
//public class BlueBackRoadRunnerTest extends LinearOpMode{
//
//    @Override
//    public void runOpMode() {
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Pose2d startPose = new Pose2d(-60,-30,Math.toRadians(0));
//        Vector2d dropPurple = new Vector2d(-30,-30);
//        Vector2d closeTruss = new Vector2d(-8,-20);
//        Vector2d farTruss = new Vector2d(-8,20);
//        Vector2d closeTruss2 = new Vector2d(-11,-20);
//        Vector2d farTruss2 = new Vector2d(-11,20);
//        Vector2d backdrop = new Vector2d(-40,63);
//        Vector2d pickup = new Vector2d(-30,-41);
//        Vector2d park = new Vector2d(-70,60);
//        drive.setPoseEstimate(startPose);
//
//
//        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
////                .splineToConstantHeading(new Vector2d(-50, -30),Math.toRadians(0))
//                .splineToConstantHeading(dropPurple, Math.toRadians(0))
//                .waitSeconds(1)
//                .splineToLinearHeading(new Pose2d(closeTruss.getX(), closeTruss.getY(), Math.toRadians(90)), Math.toRadians(0))
//                .splineToConstantHeading(farTruss, Math.toRadians(90))
//                .splineToConstantHeading(backdrop,Math.toRadians(90))
//                .waitSeconds(1)
//                .setReversed(true)
//                .splineToConstantHeading(farTruss, Math.toRadians(-90))
//                .splineToConstantHeading(closeTruss, Math.toRadians(-90))
//                .splineToConstantHeading(pickup, Math.toRadians(-90))
//                .waitSeconds(1)
//                .setReversed(false)
//                .splineToConstantHeading(closeTruss2,Math.toRadians(90))
//                .splineToConstantHeading(farTruss2, Math.toRadians(90))
//                .splineToConstantHeading(backdrop,Math.toRadians(90))
//                .waitSeconds(1)
//                .strafeTo(park)
//                .forward(10)
////                .forward(30)
//                //deposit purple pixel, next go to backdrop
////                .splineToLinearHeading(new Pose2d(-30,-34, Math.toRadians(0)), Math.toRadians(0))
////                .splineToLinearHeading(new Pose2d(-10,-20, Math.toRadians(90)), Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(-10,20), Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(-45,50), Math.toRadians(180))
////                .waitSeconds(1)
////                .setReversed(true)
////                .splineToConstantHeading(new Vector2d(-10,20), Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(-10,-20), Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(-45,-50), Math.toRadians(180))
////                .waitSeconds(1)
////                .setReversed(false)
////                .splineToLinearHeading(new Pose2d(-10,-20, Math.toRadians(90)), Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(-10,20), Math.toRadians(180))
////                .splineToConstantHeading(new Vector2d(-45,50), Math.toRadians(180))
////                .waitSeconds(1)
////                .strafeTo(new Vector2d(-55, 50))
////                .waitSeconds(200)
////
////
////                .splineToConstantHeading(new Vector2d(10,-50), Math.toRadians(0))
////                .splineToLinearHeading(new Pose2d(50,-50, Math.toRadians(90)), Math.toRadians(0))
////                .waitSeconds(2)
////                //deposit yellow pixel
////                .splineToConstantHeading(new Vector2d(70,-27), Math.toRadians(180))
////                .back(60)
////                .waitSeconds(10)
////                .splineToConstantHeading(new Vector2d(-57,35), Math.toRadians(180))
////                //pickup white pixels
////                .waitSeconds(2)
////                .setReversed(true)
////                .splineToConstantHeading(new Vector2d(-35,11), Math.toRadians(0))
////                .setReversed(false)
////                .forward(60)
////                .splineToConstantHeading(new Vector2d(47,35), Math.toRadians(0))
////                .waitSeconds(2)
////                //deposit 2 white pixels
////                .strafeTo(new Vector2d(47,60))
////                .strafeTo(new Vector2d(60,60))
////                .turn(Math.toRadians(90))
////                .waitSeconds(2)
////                //park
//                .build();
//
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        drive.followTrajectorySequence(ts);
//    }
//
//}
