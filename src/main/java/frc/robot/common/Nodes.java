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
                "P 1",
                "P 2",
                "P 3",
                "P 4",
                "P 5",
                "P 6"
        };
        translations = new Translation2d[] {
                new Translation2d(15, 0.54),
                new Translation2d(15, 1.70),
                new Translation2d(15, 2.13),
                new Translation2d(15, 3.31),
                new Translation2d(15, 3.84),
                new Translation2d(15, 5.05)
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
