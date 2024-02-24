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


import static org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode.setOpMode;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.derivative;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.integral;
import static org.firstinspires.ftc.teamcode.AAAOpModes.TeleOp.Dashboard.proportional;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Vision.BluePipeline;
import org.firstinspires.ftc.teamcode.Vision.RedPipeline;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.Control.PID;
import org.firstinspires.ftc.teamcode.zLibraries.Utilities.HardwareDevices.Gyro;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Red 50 Point Front Auto", group="Linear OpMode")
//@Disabled
public class Red50Point extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    Servo purple;
    Servo depositer;
    Servo depositerDoor;
    DcMotor slideMotorL;
    DcMotor slideMotorR;
    DcMotor intakeMotor;
    CRServo counterRoller;
    ElapsedTime time = new ElapsedTime();

    int slidePosition = 0;
    Gyro gyro;


    PID pid;


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private OpenCvCamera camera;
    RedPipeline pipeline = new RedPipeline();


    //BluePipeline pipeline = new BluePipeline();
    @Override
    public void runOpMode() {
        //TODO ADD THIS LINE TO EVERY OPMODE
        setOpMode(this);
        //TODO THAT ONE ^
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
        purple = BaseOpMode.hardware.servo.get("purple");
        depositer = BaseOpMode.hardware.servo.get("depositer");
        depositerDoor = BaseOpMode.hardware.servo.get("depositerDoor");
        counterRoller = BaseOpMode.hardware.crservo.get("roll");
        slideMotorL = BaseOpMode.hardware.get(DcMotor.class,"slideL");
        slideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorL.setTargetPosition(0);
        slideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorL.setPower(0.5);
        slideMotorR = BaseOpMode.hardware.get(DcMotor.class,"slideR");
        slideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorR.setTargetPosition(0);
        slideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorR.setPower(0.5);
        intakeMotor = BaseOpMode.hardware.get(DcMotor.class,"intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setTargetPosition(0);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(0.2);


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


        double fast = 0.4;

        if (opModeIsActive()) {
            //CODE THAT RUNS AFTER START

            int teamPropPosition = 0;
            while (teamPropPosition == 0) {
                teamPropPosition = pipeline.getRectPos();
            }
            drive(200, -1, 0, 0, fast);
            drive(400, 0, 1, 0, fast);
            if (teamPropPosition == 1) {
                //left position

                telemetry.addData("position", "left");
                telemetry.update();
                drive(100, 0, 0, 1, fast);
                drive(600, -1, 0, 0, fast);
                drive(750, 0, 0, -1, fast);
                drive(200, -1, 0, 0, fast);
                //Deposit Purple Pixel
                time.reset();
                while (time.seconds() <= 1) {
                    purple.setPosition(0.35);
                }
                drive(300, 1, 0, 0, fast);
                drive(725, 0, 0, 1, fast);

            } else if (teamPropPosition == 2) {
                //middle position
                telemetry.addData("position", "middle");
                telemetry.update();
                drive(580, -1, 0, 0, fast);
                drive(100, 0, 1, 0, fast);
                //Deposit Purple Pixel
                time.reset();
                while (time.seconds() <= 1) {
                    purple.setPosition(0.4);
                }
                drive(115, 1, 0, 0, fast);

            } else if (teamPropPosition == 3) {
                //right position
                telemetry.addData("position", "right");
                telemetry.update();
                drive(180, 0, 1, 0, fast);
                drive(150, 0, 0, 1, fast);
                drive(200, -1, 0, 0, fast);
                //Deposit Purple Pixel
                time.reset();
                while (time.seconds() <= 1) {
                    purple.setPosition(0.4);
                }
                drive(200, 1, 0, 0, fast);

            }

            if (teamPropPosition == 3) {
                drive(100, 1, 0, 0, fast);
                drive(600, 0, 0, 1, fast);
                drive(900, -1, 0, 0, fast);
            } else if (teamPropPosition == 2) {
                drive(850, 0, 0, 1, fast);
                drive(900, -1, 0, 0, fast);
            } else {
//            drive(550, -1, 0, 0, fast);
                drive(800, 0, 0, 1, fast);
                drive(900, -1, 0, 0, fast);
            }


            if (teamPropPosition == 1) {
                //left position
                telemetry.addData("position","left");
                telemetry.update();
                drive(300, -1, 0, 0, fast);
                drive(300, 0, -1, 0, fast);
                drive(100, 0, 0, -1, fast);
                drive(200, -1, 0, 0, fast);
                // Deposit yellow Pixel
                depositer.setPosition(0.85);
                time.reset();
                while (time.seconds() <= 3) {
                    if (time.seconds() > 2) {
                        slidePosition = -700;
                    }else if (time.seconds() > 1) {
                        depositerDoor.setPosition(0);
                        depositer.setPosition(0.55);
                    }else  if (time.seconds() > 0) {
                        depositer.setPosition(0.85);
                        slidePosition = -1000;
                    }
                    slideMotorR.setTargetPosition(slidePosition);
                    slideMotorL.setTargetPosition(slidePosition);
                }
                depositerDoor.setPosition(0.07);
                time.reset();
                while (time.seconds() <= 3) {
                    if (time.seconds() > 2) {
                        depositer.setPosition(0.85);
                        slidePosition = 10;
                    }
                    else if (time.seconds() > 1) {
                        depositerDoor.setPosition(0.15);
                        slidePosition = -1000;
                    }
                    else if (time.seconds() > 0) {
                        depositerDoor.setPosition(0.07);
                    }
                    slideMotorR.setTargetPosition(slidePosition);
                    slideMotorL.setTargetPosition(slidePosition);
                }
                drive(200, 1, 0, 0, fast);
                drive(750, 0, 0, 1, fast);
                drive(1700, -1, 0, 0, fast);

            } else if (teamPropPosition == 2) {
                //middle position
                telemetry.addData("position","middle");
                telemetry.update();
                drive(350, -1, 0, 0, fast);
                drive(200, 0, -1, 0, fast);
                drive(300, -1, 0, 0, fast);
                // Deposit yellow Pixel
                depositer.setPosition(0.85);
                time.reset();
                while (time.seconds() <= 3) {
                    if (time.seconds() > 2) {
                        slidePosition = -700;
                    }else if (time.seconds() > 1) {
                        depositerDoor.setPosition(0);
                        depositer.setPosition(0.55);
                    }else  if (time.seconds() > 0) {
                        depositer.setPosition(0.85);
                        slidePosition = -1000;
                    }
                    slideMotorR.setTargetPosition(slidePosition);
                    slideMotorL.setTargetPosition(slidePosition);
                }
                depositerDoor.setPosition(0.07);
                time.reset();
                while (time.seconds() <= 3) {
                    if (time.seconds() > 2) {
                        depositer.setPosition(0.85);
                        slidePosition = 10;
                    }
                    else if (time.seconds() > 1) {
                        depositerDoor.setPosition(0.15);
                        slidePosition = -1000;
                    }
                    else if (time.seconds() > 0) {
                        depositerDoor.setPosition(0.07);
                    }
                    slideMotorR.setTargetPosition(slidePosition);
                    slideMotorL.setTargetPosition(slidePosition);
                }
                drive(200, 1, 0, 0, fast);
                drive(750, 0, 0, 1, fast);
                drive(1100, -1, 0, 0, fast);

            } else if (teamPropPosition == 3) {
                //right position
                telemetry.addData("position","right");
                telemetry.update();
                drive(400, 0, -1, 0, fast);
//                drive(300, 0, 0, -1, fast);
                drive(300, -1, 0, 0, fast);
                // Deposit yellow Pixel
                depositer.setPosition(0.85);
                time.reset();
                while (time.seconds() <= 3) {
                    if (time.seconds() > 2) {
                        slidePosition = -700;
                    }else if (time.seconds() > 1) {
                        depositerDoor.setPosition(0);
                        depositer.setPosition(0.55);
                    }else  if (time.seconds() > 0) {
                        depositer.setPosition(0.85);
                        slidePosition = -1000;
                    }
                    slideMotorR.setTargetPosition(slidePosition);
                    slideMotorL.setTargetPosition(slidePosition);
                }
                depositerDoor.setPosition(0.07);
                time.reset();
                while (time.seconds() <= 3) {
                    if (time.seconds() > 2) {
                        depositer.setPosition(0.85);
                        slidePosition = 10;
                    }
                    else if (time.seconds() > 1) {
                        depositerDoor.setPosition(0.15);
                        slidePosition = -1000;
                    }
                    else if (time.seconds() > 0) {
                        depositerDoor.setPosition(0.07);
                    }
                    slideMotorR.setTargetPosition(slidePosition);
                    slideMotorL.setTargetPosition(slidePosition);
                }
                drive(200, 1, 0, 0, fast);
                drive(750, 0, 0, 1, fast);
                drive(700, -1, 0, 0, fast);

            }

            telemetry.update();

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
            distanceDriven = (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs((br.getCurrentPosition()) * 11/12) + Math.abs((bl.getCurrentPosition()) * 11/12)) /4;
            variable = variable + 0.1;
            //telemetry.addData("distancedriven", distanceDriven);
            //telemetry.addData("drive", drive);
            //telemetry.addData("strafe", strafe);
            //telemetry.addData("turn", turn);
            //telemetry.addData("speed", speed);



//            fl.setPower(0.5);
//            fr.setPower(0.5);
//            bl.setPower(0.5);
//            br.setPower(0.5);
//
            fl.setPower((drive - strafe + turn) * speed);
            fr.setPower((drive + strafe - turn) * speed);
            bl.setPower(((drive + strafe + turn) * speed) * 11/12);
            br.setPower(((drive - strafe - turn) * speed) * 11/12);








        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);




    }






}

