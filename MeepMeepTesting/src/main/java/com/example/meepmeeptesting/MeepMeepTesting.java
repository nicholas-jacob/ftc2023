package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(12.47, 17.375)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(24, 20, Math.toRadians(160), Math.toRadians(140), 15.8)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(36, -61.3125, Math.toRadians(90)))
                                        .lineTo(new Vector2d(36,-24))
                                        .splineTo(new Vector2d(49.5, -14), Math.toRadians(-12.0362))//og angle is 194.0362
                                        .splineToConstantHeading(new Vector2d(58, -6.5), Math.toRadians(90-14.0362))
                                        .lineToSplineHeading(new Pose2d(61, -11.75, Math.toRadians(0)))
//                                        .lineToConstantHeading(new Vector2d(35.5, -11.75))
//set up two inches away from the wall
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}