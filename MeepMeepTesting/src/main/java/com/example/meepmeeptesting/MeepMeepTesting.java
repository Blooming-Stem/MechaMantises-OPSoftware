package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(35, 35, Math.toRadians(281.5), Math.toRadians(222.7), 12)
                .followTrajectorySequence(drive ->


                       drive.trajectorySequenceBuilder(new Pose2d(35,60,Math.toRadians(90)))


                        .addDisplacementMarker(() -> {
                           // targets = 2950;
                        })
                        .back(57)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                           // turret.setPosition(0.1);
                        })
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                            //claw.setPosition(0.3);
                        })
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> {
                            //turret.setPosition(0.5);
                           // targets = 500;
                        })
                        .forward(9)
                        .turn(Math.toRadians(-90))
                        .forward(24)
                        .addTemporalMarker(() -> {
                            //claw.setPosition(1);
                        })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                            //targets = 2950;
                        })
                        .waitSeconds(0.2)
                        .back(32)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                            //turret.setPosition(0.9);
                        })
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                           // claw.setPosition(0.3);
                        })
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> {
                           // turret.setPosition(0.5);
                           // targets = 400;
                        })
                        .forward(32.5)
                        .addTemporalMarker(() -> {
                            //claw.setPosition(1);
                        })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                          //  targets = 2950;
                        })
                        .waitSeconds(0.2)
                        .back(32)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                           // turret.setPosition(0.9);
                        })
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                          //  claw.setPosition(0.3);
                        })
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> {
                          //  turret.setPosition(0.5);
                           // targets = 300;
                        })
                        .forward(32.5)
                        .addTemporalMarker(() -> {
                          //  claw.setPosition(1);
                        })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                           // targets = 2950;
                        })
                        .waitSeconds(0.2)
                        .back(32)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                          //  turret.setPosition(0.9);
                        })
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                           // claw.setPosition(0.3);
                        }).waitSeconds(0.5)
                        .addTemporalMarker(() -> {
                           // turret.setPosition(0.5);
                           // targets = 200;
                        })
                        .forward(32.5)
                        .addTemporalMarker(() -> {
                          //  claw.setPosition(1);
                        })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    targets=700;
//                })
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                           // targets = 2950;
                        })
                        .waitSeconds(0.2)
                        .back(32)
                        .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                           // turret.setPosition(0.9);
                        })
                        .waitSeconds(0.2)
                        .addTemporalMarker(() -> {
                          //  claw.setPosition(0.3);
                        })
                        .waitSeconds(0.5)
                        .addTemporalMarker(() -> {
                          //  turret.setPosition(0.5);
                           // targets = 0;
                        })
                        .forward(28)


                        .build()
                );





        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)

                .start();
    }
}










