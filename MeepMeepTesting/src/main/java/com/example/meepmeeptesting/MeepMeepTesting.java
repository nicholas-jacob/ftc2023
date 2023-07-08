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
                .setConstraints(50, 50, Math.toRadians(160), Math.toRadians(140), 15.8)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-33, -64.065, Math.toRadians(180)))
                                        .lineToConstantHeading(new Vector2d(-34.5,-46))
                                        .splineToSplineHeading(new Pose2d(-36.5,-24, Math.toRadians(110)), Math.toRadians(110))
                                        .splineTo(new Vector2d(-49.5, -14), Math.toRadians(194.0362))//og angle is 194.0362
                                        .splineToConstantHeading(new Vector2d(-57, -6.75), Math.toRadians(90+14.0362))
                                        .lineToSplineHeading(new Pose2d(-59, -11.75, Math.toRadians(180)))
                                        .lineToConstantHeading(new Vector2d(-35, -11.75))
                                        .lineToConstantHeading(new Vector2d(-11.5, -11.75))
                                        .lineToConstantHeading(new Vector2d(11.5, -11.75))
                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}