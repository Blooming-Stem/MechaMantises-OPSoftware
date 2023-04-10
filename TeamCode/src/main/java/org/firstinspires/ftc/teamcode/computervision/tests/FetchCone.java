

package org.firstinspires.ftc.teamcode.computervision.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.computervision.CVMaster;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous
public class FetchCone extends LinearOpMode
{
    private double distance;
    private double lateral;
    private double angle;
    private Servo claw;
    private Servo turret;





    @Override
    public void runOpMode()
    {
        claw = hardwareMap.servo.get("claw");
        claw.setPosition(0.6);
        turret = hardwareMap.servo.get("turret");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */



        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        CVMaster cv = new CVMaster(this);
        cv.observeStick();
        while(!isStarted()&&!isStopRequested()) {

            if (cv.pipeline.pixel_height > 0) {
                telemetry.addLine("Ready!");
                telemetry.addData("Distance from Cone", cv.pipeline.getDistanceFromCone());
                telemetry.addData("Lateral Distance from Cone", cv.pipeline.getlateralDistance());
                telemetry.addData("Angle from Cone", cv.pipeline.turnAngle());
            } else {

            }
            telemetry.update();
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        distance = cv.pipeline.getDistanceFromCone();
        lateral = cv.pipeline.getlateralDistance();
        angle = cv.pipeline.turnAngle();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis",distance);
        telemetry.update();



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */


        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0,0))
                .turn(Math.toRadians(180))


                .lineToLinearHeading(new Pose2d(-distance-5,lateral, Math.toRadians(180)))
                .addTemporalMarker(()->{
                    claw.setPosition(1);
                })
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(0,0))
                .build();


        drive.followTrajectorySequence(traj);


        cv.stopCamera();
    }
}