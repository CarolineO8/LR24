//package org.firstinspires.ftc.teamcode.KCP.TestOpModes;
//
//import static org.firstinspires.ftc.teamcode.AAAOpModes.Testing.BotTestingTeleOp.AAATestDash.loopTime;
//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.multTelemetry;
//import static org.firstinspires.ftc.teamcode.zLibraries.Utilities.OpModeUtils.setWolfpackOpMode;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.AAAOpModes.Templates.WolfpackOpMode;
//import org.firstinspires.ftc.teamcode.KCP.Movement;
//import org.firstinspires.ftc.teamcode.Subsystems.ClawIntake;
//import org.firstinspires.ftc.teamcode.Subsystems.Depositor;
//
//@TeleOp(name="HoldPos Test", group="Testing")
//public class HoldPosTest extends WolfpackOpMode {
//
//    public Movement movement;
//    public ClawIntake intake;
//    public Depositor depositor;
//    public ElapsedTime runtime = new ElapsedTime();
//
//    @Override
//    public void externalInit() {
//        setWolfpackOpMode(this);
//        intake = new ClawIntake();
//        depositor = new Depositor();
//        movement = new Movement(0, 0);
//    }
//
//    @Override
//    public void externalInitLoop() {
//        intake.setV4BPosition(ClawIntake.ClawIntakeDash.v4bUprightIntake);
//    }
//
//    @Override
//    public void externalStart() {
//
//    }
//
//    @Override
//    public void externalLoop() {
//        if(runtime.milliseconds() > loopTime) {
//            runtime.reset();
//            movement.update();
//            movement.holdPosition(0, 0, 0);
//            multTelemetry.update();
//        }
//    }
//
//    @Override
//    public void externalStop() {
//        movement.getGyro().imuThread.interrupt();
//    }
//}
