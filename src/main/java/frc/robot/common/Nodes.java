package frc.robot.common;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Swerve;

public class Nodes {
    private static String[] redTranslationLabels;
    private static String[] blueTranslationLabels;
    private static Translation2d[] redTranslations;
    private static Translation2d[] blueTranslations;

    private Swerve swerve;

    public Nodes(Swerve swerve) {
        this.swerve = swerve;

        redTranslationLabels = new String[] {
                "RP 1",
                "RP 2",
                "RP 3",
                "RP 4",
                "RP 5",
                "RP 6"
        };

        blueTranslationLabels = new String[] {
                "BP 1",
                "BP 2",
                "BP 3",
                "BP 4",
                "BP 5",
                "BP 6"
        };

        redTranslations = new Translation2d[] {

                new Translation2d(14.9, 0.54),
                new Translation2d(14.93, 1.65),
                new Translation2d(14.84, 2.23),
                new Translation2d(14.85, 3.38),
                new Translation2d(14.87, 3.87),
                new Translation2d(14.87, 5.01),
        };
        blueTranslations = new Translation2d[] {

                new Translation2d(1.64, 4.97),
                new Translation2d(1.64, 3.84),
                new Translation2d(1.64, 3.33),
                new Translation2d(1.64, 2.19),
                new Translation2d(1.64, 1.63),
                new Translation2d(1.64, 0.5),
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
