package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class autonew extends LinearOpMode {
    private PIDController slidescontroller;
    public static double ps = 0.002, is = 0, ds = 0;
    public static double fs = 0.01;
    public static int targets = 0;
    private final double ticks_in_degrees = 384 / 360.0;
    private DcMotorEx slidesleft;
    private DcMotorEx slidesright;
    private Servo turret;
    private Servo claw;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.03175;

    // int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;
    int left = 1;
    int middle = 2;
    int right = 3;




    SampleMecanumDrive drive;
    public void runOpMode() {
        slidescontroller = new PIDController(ps, is, ds);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slidesleft = hardwareMap.get(DcMotorEx.class, "slidesLeft");
        slidesright = hardwareMap.get(DcMotorEx.class, "slidesRight");

        slidesleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slidesright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesright.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");
        turret = hardwareMap.servo.get("turret");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);


        claw.setPosition(1);
        turret.setPosition(0.5);
//        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())
//
//
//                .addDisplacementMarker(() -> {
//                    targets = 2950;
//                })
//                .back(57)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.1);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 500;
//                })
//                .forward(8)
//                .turn(Math.toRadians(-90))
//                .forward(25.5)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    targets = 2950;
//                })
//                .waitSeconds(0.2)
//                .back(32.75)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 400;
//                })
//                .forward(32.75)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    targets = 2950;
//                })
//                .waitSeconds(0.2)
//                .back(32.75)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 300;
//                })
//                .forward(32.75)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    targets = 2950;
//                })
//                .waitSeconds(0.2)
//                .back(32.75)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                }).waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 200;
//                })
//                .forward(32.75)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    targets = 2950;
//                })
//                .waitSeconds(0.2)
//                .back(32.75)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 0;
//                })
//                .forward(28)
//
//
//                .build();
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d())


                .addDisplacementMarker(() -> {
                    targets = 2950;
                })
                .back(56)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.1);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 500;
                })
                .forward(7)
                .turn(Math.toRadians(-87.5))
                .forward(27)
                .addTemporalMarker(() -> {
                    claw.setPosition(1);
                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    targets = 2950;
                })
                .waitSeconds(0.2)
                .back(33.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.9);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 400;
                })
                .forward(34)
                .addTemporalMarker(() -> {
                    claw.setPosition(1);
                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    targets = 2950;
                })
                .waitSeconds(0.2)
                .back(33.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.9);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 300;
                })
                .forward(34)
                .addTemporalMarker(() -> {
                    claw.setPosition(1);
                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    targets = 2950;
                })
                .waitSeconds(0.2)
                .back(33.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.9);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                }).waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 0;
                })

                .forward(34)


                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d())


                .addDisplacementMarker(() -> {
                    targets = 2950;
                })
                .back(56)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.1);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 500;
                })
                .forward(7)
                .turn(Math.toRadians(-87.5))
                .forward(27)
                .addTemporalMarker(() -> {
                    claw.setPosition(1);
                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    targets = 2950;
                })
                .waitSeconds(0.2)
                .back(33.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.9);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 400;
                })
                .forward(34)
                .addTemporalMarker(() -> {
                    claw.setPosition(1);
                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    targets = 2950;
                })
                .waitSeconds(0.2)
                .back(33.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.9);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 300;
                })
                .forward(34)
                .addTemporalMarker(() -> {
                    claw.setPosition(1);
                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    targets = 2950;
                })
                .waitSeconds(0.2)
                .back(33.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.9);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                }).waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 0;
                })

                .forward(6)


                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d())


                .addDisplacementMarker(() -> {
                    targets = 2950;
                })
                .back(56)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.1);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 500;
                })
                .forward(7)
                .turn(Math.toRadians(-87.5))
                .forward(27)
                .addTemporalMarker(() -> {
                    claw.setPosition(1);
                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    targets = 2950;
                })
                .waitSeconds(0.2)
                .back(33.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.9);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 400;
                })
                .forward(34)
                .addTemporalMarker(() -> {
                    claw.setPosition(1);
                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    targets = 2950;
                })
                .waitSeconds(0.2)
                .back(33.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.9);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 300;
                })
                .forward(34)
                .addTemporalMarker(() -> {
                    claw.setPosition(1);
                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    targets = 2950;
                })
                .waitSeconds(0.2)
                .back(33.5)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    turret.setPosition(0.9);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0.5);
                }).waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    turret.setPosition(0.5);
                    targets = 0;
                })

                .back(16)


                .build();

//        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d())
//
//
//                .addDisplacementMarker(() -> {
//                    targets = 2950;
//                })
//                .back(57)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.1);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 500;
//                })
//                .forward(8)
//                .turn(Math.toRadians(-90))
//                .forward(25.5)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    targets = 2950;
//                })
//                .waitSeconds(0.2)
//                .back(32.75)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 400;
//                })
//                .forward(32.75)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    targets = 2950;
//                })
//                .waitSeconds(0.2)
//                .back(32.75)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 300;
//                })
//                .forward(32.75)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    targets = 2950;
//                })
//                .waitSeconds(0.2)
//                .back(32.75)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                }).waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 200;
//                })
//                .forward(32.75)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(1);
//                })
////                .waitSeconds(0.3)
////                .addTemporalMarker(()->{
////                    targets=700;
////                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    targets = 2950;
//                })
//                .waitSeconds(0.2)
//                .back(32.75)
//                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
//                    turret.setPosition(0.9);
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    claw.setPosition(0.3);
//                })
//                .waitSeconds(0.5)
//                .addTemporalMarker(() -> {
//                    turret.setPosition(0.5);
//                    targets = 0;
//                })
//                .back(20)
//
//
//                .build();
//        //drive.followTrajectorySequenceAsync(traj1);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "MantisCam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        //  MantisClass mantis = new MantisClass(this);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(camera, 10);
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left||tag.id ==middle||tag.id==right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */


        if(tagOfInterest ==null){
            drive.followTrajectorySequenceAsync(traj1);

        }   else {
            switch (tagOfInterest.id) {
                case 1:
                    drive.followTrajectorySequenceAsync(traj1);
                    break;
                case 2:
                    drive.followTrajectorySequenceAsync(traj2);
                    break;
                case 3:
                    drive.followTrajectorySequenceAsync(traj3);
                    break;
            }
        }






        while(opModeIsActive()) {
            drive.update();

            slidescontroller.setPID(ps, is, ds);
            int slidesleftpos = slidesleft.getCurrentPosition();
            double pidleft = slidescontroller.calculate(slidesleftpos, targets);
            double ffleft = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

            int slidesrightpos = slidesright.getCurrentPosition();
            double pidright = slidescontroller.calculate(slidesrightpos, targets);
            double ffright = Math.cos(Math.toRadians(targets / ticks_in_degrees)) * fs;

            double powerleft = pidleft + ffleft;

            double powerright = pidright + ffright;

            slidesleft.setPower(powerleft);
            slidesright.setPower(powerright);


        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
