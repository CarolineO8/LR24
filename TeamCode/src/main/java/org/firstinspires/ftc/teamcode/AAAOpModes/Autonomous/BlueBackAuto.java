/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous;


import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.derivative;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.integral;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.proportional;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.BluePipeline;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Gyro;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Blue Back Auto: Linear OpMode", group="Linear OpMode")
//@Disabled
public class BlueBackAuto extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    Gyro gyro;


    PID pid;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private OpenCvCamera camera;
    //RedPipeline pipeline = new RedPipeline();
    BluePipeline pipeline = new BluePipeline();
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //CODE THAT RUNS AFTER INIT
        fl = hardwareMap.get(DcMotor.class,"fl");
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        fr = hardwareMap.get(DcMotor.class,"fr");
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        br = hardwareMap.get(DcMotor.class,"br");
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        bl = hardwareMap.get(DcMotor.class,"bl");
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        gyro = new Gyro(hardwareMap);
        pid = new PID(proportional,integral,derivative);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // Start streaming
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);


//this just sets up the camera
                //  camera.resumeViewport();


            }


            @Override
            public void onError(int errorCode) {
//When camera doesn't work nothing happens
            }
        });
        camera.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(camera, 30);




        waitForStart();
        resetRuntime();
        runtime.reset();




        if (opModeIsActive()) {
            boolean restart = true;
            //CODE THAT RUNS AFTER START
            while(restart && opModeIsActive()) {
                int teamPropPosition = pipeline.getRectPos();
                drive(100, 0, 1, 0, 0.4);

                while (restart) {
                    teamPropPosition = pipeline.getRectPos();
                    if (teamPropPosition == 1) {
                        //left position

                        telemetry.addData("position", "left");
                        telemetry.update();
                        drive(1150, -1, 0, 0, 0.4);
                        drive(750, 0, 0, 1, 0.2);
                        // Deposit Purple Picture
                        drive(725, 0, 0, -1, 0.2);
                        restart = false;


                    } else if (teamPropPosition == 2) {
                        //middle position
                        telemetry.addData("position", "middle");
                        telemetry.update();
                        drive(1150, -1, 0, 0, 0.4);
                        restart = false;


                    } else if (teamPropPosition == 3) {
                        //right position
                        telemetry.addData("position", "right");
                        telemetry.update();
                        drive(1150, -1, 0, 0, 0.4);
                        drive(750, 0, 0, -1, 0.2);
                        // Deposit Purple Picture
                        drive(725, 0, 0, 1, 0.2);
                        restart = false;
                    }
                }

                drive(800, -1, 0, 0, 0.4);
                if (teamPropPosition == 2) {
                    //Deposit Purple Pixel
                }
                drive(800, 0, 0, -1, 0.2);
                drive(3500, -1, 0, 0, 0.4);
                drive(50, 0, 0, -1, 0.4);
                drive(1100, 0, -1, 0, 0.4);


                if (teamPropPosition == 1) {
                    //left position


                    telemetry.addData("position","left");
                    telemetry.update();
                    drive(600, 0, -1, 0, 0.4);
                    drive(400, -1, 0, 0, 0.4);
                    // Deposit yellow Picture
                    drive(200, 1, 0, 0, 0.4);
                    drive(1600, 0, 1, 0, 0.4);
                    drive(400, -1, 0, 0, 0.4);





                } else if (teamPropPosition == 2) {
                    //middle position
                    telemetry.addData("position","middle");
                    telemetry.update();
                    drive(400, -1, 0, 0, 0.4);
                    // Deposit yellow Picture
                    drive(200, 1, 0, 0, 0.4);
                    drive(1000, 0, 1, 0, 0.4);
                    drive(600, -1, 0, 0, 0.4);



                } else if (teamPropPosition == 3) {
                    //right position
                    telemetry.addData("position","right");
                    telemetry.update();
                    drive(600, 0, 1, 0, 0.4);
                    drive(400, -1, 0, 0, 0.4);
                    // Deposit yellow Picture
                    drive(200, 1, 0, 0, 0.4);
                    drive(400, 0, 1, 0, 0.4);
                    drive(600, -1, 0, 0, 0.4);

                }


                telemetry.addData("restart",restart);
                telemetry.update();
            }
        }


    }




    public void drive(double distance, double drive, double strafe, double turn, double speed){
        double distanceDriven = 0;
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        double variable = 0;


        while (Math.abs(distanceDriven) <= distance) {
            distanceDriven = (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(br.getCurrentPosition()) + Math.abs(bl.getCurrentPosition())) /4;
            variable = variable + 0.1;
            //telemetry.addData("distancedriven", distanceDriven);
            //telemetry.addData("drive", drive);
            //telemetry.addData("strafe", strafe);
            //telemetry.addData("turn", turn);
            //telemetry.addData("speed", speed);
            telemetry.addData("variable", variable);
            telemetry.update();




//            fl.setPower(0.5);
//            fr.setPower(0.5);
//            bl.setPower(0.5);
//            br.setPower(0.5);
//
            fl.setPower((drive - strafe + turn) * speed);
            fr.setPower((drive + strafe - turn) * speed);
            bl.setPower((drive + strafe + turn) * speed);
            br.setPower((drive - strafe - turn) * speed);








        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);




    }






}

