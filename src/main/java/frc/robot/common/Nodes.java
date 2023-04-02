package frc.robot.common;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;

public class Nodes {
    private static String[] translationLabels;
    private static Translation2d[] translations;

    private Swerve swerve;

    public Nodes(Swerve swerve) {
        this.swerve = swerve;

        translationLabels = new String[] {
                "RP 1",
                "RP 2",
                "RP 3",
                "RP 4",
                "RP 5",
                "RP 6",
                "BP 1",
                "BP 2",
                "BP 3",
                "BP 4",
                "BP 5",
                "BP 6"
        };
        translations = new Translation2d[] {

                new Translation2d(14.9, 0.54),
                new Translation2d(14.93, 1.65),
                new Translation2d(14.84, 2.23),
                new Translation2d(14.85, 3.38),
                new Translation2d(14.87, 3.87),
                new Translation2d(14.87, 5.01),

                new Translation2d(1.64, 4.97),
                new Translation2d(1.64, 3.84),
                new Translation2d(1.64, 3.33),
                new Translation2d(1.64, 2.19),
                new Translation2d(1.64, 1.63),
                new Translation2d(1.64, 0.5),
        };
    }

    public Translation2d[] getTranslations() {
        return translations;
    }

    public Translation2d getNearestTranslation() {
        Translation2d nearestNode = translations[0];
        for (Translation2d translation : translations) {
            if (DistanceToTranslation(translation) < DistanceToTranslation(nearestNode)) {
                nearestNode = translation;
            }
        }

        return nearestNode;
    }

    public String getTranslationLabel(Translation2d translation) {
        for (int i = 0; i < translationLabels.length; i++) {
            if (translations[i] == translation) {
                return translationLabels[i];
            }
        }

        return "";
    }

    private double DistanceToTranslation(Translation2d translation) {
        return DistanceBetweenTranslations(swerve.getPose().getTranslation(), translation);
    }

    private double DistanceBetweenTranslations(Translation2d translationA, Translation2d translationB) {
        return Math.sqrt(Math.pow(translationA.getX() - translationB.getX(), 2)
                + Math.pow(translationA.getY() - translationB.getY(), 2));
    }
}
