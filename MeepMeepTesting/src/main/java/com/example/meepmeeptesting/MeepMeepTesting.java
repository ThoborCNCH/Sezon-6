package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.3046821376492, 52.48291908330528, Math.toRadians(242.8113), Math.toRadians(246.4794885245902), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-65, 35, Math.toRadians(180)))
                                .turn(Math.toRadians(180))
                                .forward(50)
                                .back(24)
                                .turn(Math.toRadians(45))
                                .addTemporalMarker(1, () -> {
//                                    robot.setGliseraPower(1);
                                })
                                .addTemporalMarker(1, () -> {
//                                    robot.setIntake(-1);
                                })
                                .forward(12)
                                .addTemporalMarker(1, () -> {
//                                    robot.setIntake(1);
                                })
                                .back(12)
                                .addTemporalMarker(1, () -> {
//                                    robot.setGliseraPower(-1);
                                })
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}