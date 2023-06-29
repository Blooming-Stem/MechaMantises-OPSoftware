package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDriveLock;

@Autonomous
public class Poslock extends LinearOpMode {
    public void runOpMode(){
        SampleMecanumDriveLock bot = new SampleMecanumDriveLock(hardwareMap);
        while(!opModeIsActive()&&!isStopRequested()){
            telemetry.addData("Ready", "yah");
            telemetry.update();

        }
        waitForStart();
        Trajectory lock = bot.trajectoryBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(0.1,0,0))
                .build();

        bot.followTrajectory(lock);
    }

}
