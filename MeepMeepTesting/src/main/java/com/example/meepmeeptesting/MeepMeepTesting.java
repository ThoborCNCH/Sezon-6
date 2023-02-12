package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.8)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38, -57.8, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-38, -40, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-30, -7, Math.toRadians(180)))
                                .strafeLeft(2)
                                .splineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(180))
                                .forward(2)
                                .lineToLinearHeading(new Pose2d(-30, -7, Math.toRadians(180)))
                                .strafeLeft(2)
                                .splineToLinearHeading(new Pose2d(-60, -12, Math.toRadians(180)), Math.toRadians(180))
                                .forward(2)
                                .lineToLinearHeading(new Pose2d(-30, -7, Math.toRadians(180)))
                                .strafeLeft(4)
                                .back(18)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}