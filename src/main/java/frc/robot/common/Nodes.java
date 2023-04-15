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
                "RP 1",
                "RP 2",
                "RP 3",
                "RP 4",
                "RP 5",
                "RP 6",
                "PUR 1",
                "PUR 2",
                "PUR 3"
        };

        blueTranslationLabels = new String[] {
                "BP 1",
                "BP 2",
                "BP 3",
                "BP 4",
                "BP 5",
                "BP 6",
                "PUR 1",
                "PUR 2",
                "PUR 3"
        };

        redTranslations = new Translation2d[] {

                new Translation2d(14.92, 0.51), // Yellow 1
                new Translation2d(14.93, 1.63), // Yellow 2
                new Translation2d(14.90, 2.20), // Yellow 3
                new Translation2d(14.92, 3.32), // Yellow 4
                new Translation2d(14.90, 3.90), // Yellow 5
                new Translation2d(14.91, 4.98), // Yellow 6
                new Translation2d(14.87, 1.1), // purple 1
                new Translation2d(14.87, 2.76), // purple 2
                new Translation2d(14.87, 4.44), // purple 3
        };
        blueTranslations = new Translation2d[] {

                new Translation2d(1.63, 5.01), // yellow 1
                new Translation2d(1.68, 3.85), // yellow 2
                new Translation2d(1.63, 3.29), // yelllow 3
                new Translation2d(1.60, 2.19), // yellow 4
                new Translation2d(1.64, 1.60), // yellow 5
                new Translation2d(1.63, 0.51), // yellow 6
                new Translation2d(1.64, 4.4), // purple 1
                new Translation2d(1.64, 2.74), // purple 2
                new Translation2d(1.64, 1.05), // purple 3
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
