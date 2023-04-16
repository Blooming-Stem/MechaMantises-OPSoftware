package org.firstinspires.ftc.teamcode.computervision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.computervision.pipelines.StickObserverPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CVMaster {
    OpenCvWebcam webcam;
    public StickObserverPipeline pipeline;
    private LinearOpMode op;
    public CVMaster(LinearOpMode p_op){
        //you can input  a hardwareMap instead of linearOpMode if you want
        op = p_op;

        //initialize webcam
        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "MantisCam"), cameraMonitorViewId);
    }
    public void observeStick(){
        //create the pipeline
        pipeline = new StickObserverPipeline();
        webcam.setPipeline(pipeline);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {}
        });

    }



    //stop streaming
    public void stopCamera(){
        webcam.stopStreaming();
    }


}