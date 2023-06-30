package frc.robot.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Swerve;

public class Nodes {
    private static String[] redTranslationLabels;
    private static String[] blueTranslationLabels;
    private static Translation2d[] redTranslations;
    private static Translation2d[] blueTranslations;

    // TODO: Move initialization of lists above so that we can reference this in the
    // two piece auto

    private Swerve swerve;

    public Nodes(Swerve swerve) {
        this.swerve = swerve;

        redTranslationLabels = new String[] {
                "YEL 1",
                "YEL 2",
                "YEL 3",
                "YEL 4",
                "YEL 5",
                "YEL 6",
                "PUR 1",
                "PUR 2",
                "PUR 3"
        };

        blueTranslationLabels = new String[] {
                "YEL 1",
                "YEL 2",
                "YEL 3",
                "YEL 4",
                "YEL 5",
                "YEL 6",
                "PUR 1",
                "PUR 2",
                "PUR 3"
        };

        redTranslations = new Translation2d[] {

                new Translation2d(14.92, 0.54), // Yellow 1
                new Translation2d(14.93, 1.635), // Yellow 2
                new Translation2d(14.90, 2.21), // Yellow 3
                new Translation2d(14.93, 3.305), // Yellow 4
                new Translation2d(14.92, 3.89), // Yellow 5
                new Translation2d(14.92, 4.98), // Yellow 6
                new Translation2d(14.92, 1.09), // purple 1
                new Translation2d(14.92, 2.76), // purple 2
                new Translation2d(14.92, 4.44), // purple 3
        };
        blueTranslations = new Translation2d[] {

                new Translation2d(1.64, 4.97), // yellow 1
                new Translation2d(1.64, 3.85), // yellow 2
                new Translation2d(1.64, 3.28), // yelllow 3
                new Translation2d(1.61, 2.17), // yellow 4
                new Translation2d(1.63, 1.595), // yellow 5
                new Translation2d(1.62, 0.475), // yellow 6
                new Translation2d(1.64, 4.41), // purple 1
                new Translation2d(1.63, 2.72), // purple 2
                new Translation2d(1.63, 1.03), // purple 3
                // new Translation2d(13, 6), // blue HP station
        };
    }

    public Translation2d[] getTranslations(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return blueTranslations;
        } else {
            return redTranslations;
        }
    }

    public Translation2d getNearestTranslation(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            Translation2d nearestNode = blueTranslations[0];
            for (Translation2d translation : blueTranslations) {
                if (DistanceToTranslation(translation) < DistanceToTranslation(nearestNode)) {
                    nearestNode = translation;
                }
            }
            return nearestNode;
        } else {
            Translation2d nearestNode = redTranslations[0];
            for (Translation2d translation : redTranslations) {
                if (DistanceToTranslation(translation) < DistanceToTranslation(nearestNode)) {
                    nearestNode = translation;
                }
            }
            return nearestNode;
        }
    }

    public String getTranslationLabel(Translation2d translation, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            for (int i = 0; i < blueTranslationLabels.length; i++) {
                if (blueTranslations[i] == translation) {
                    return blueTranslationLabels[i];
                }
            }

            return "";
        } else {
            for (int i = 0; i < redTranslationLabels.length; i++) {
                if (redTranslations[i] == translation) {
                    return redTranslationLabels[i];
                }
            }

            return "";
        }
    }

    private double DistanceToTranslation(Translation2d translation) {
        return DistanceBetweenTranslations(swerve.getPose().getTranslation(), translation);
    }

    private double DistanceBetweenTranslations(Translation2d translationA, Translation2d translationB) {
        return Math.sqrt(Math.pow(translationA.getX() - translationB.getX(), 2)
                + Math.pow(translationA.getY() - translationB.getY(), 2));
    }
}
