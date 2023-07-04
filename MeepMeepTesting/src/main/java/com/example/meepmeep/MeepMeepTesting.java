package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep mM = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(mM)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, 60, Math.toRadians(270)))
                                .lineToLinearHeading(new Pose2d(36, 0, Math.toRadians(180)))
                                .UNSTABLE_addTemporalMarkerOffset(-0.4, ()->{ //offset needs to be tested
                                    //lslide.slideHigh();
                                })
                                .addTemporalMarker(0.8, ()->{
                                    //claw.open();
                                })
                                .splineToLinearHeading(new Pose2d(60, 12, Math.toRadians(0)), Math.toRadians(270))
                                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                                    //lslide.slideLow();
                                })
                                .addTemporalMarker(3, ()->{
                                    //claw.close();
                                })
                                .splineToLinearHeading(new Pose2d(36, 0, Math.toRadians(180)), Math.toRadians(270))
                                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                                    //lslide.slideHigh();
                                })
                                .addTemporalMarker(6, ()->{
                                    //claw.open();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                                    //lslide.slideLow();
                                })
                                .lineTo(new Vector2d(60, 12))
                                .build()
                );

        mM.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}