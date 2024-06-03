package org.firstinspires.ftc.teamcode.AAAOpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AAAOpModes.BaseOpMode;
import org.firstinspires.ftc.teamcode.Vision.BluePipeline;
import org.firstinspires.ftc.teamcode.Vision.RedPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Test Auto")
public class TestAuto extends OpMode {
    private OpenCvCamera camera;
    RedPipeline pipeline = new RedPipeline();
    //BluePipeline pipeline = new BluePipeline();


    @Override
    public void init(){
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



    }

    @Override
    public void loop(){



    }
}
