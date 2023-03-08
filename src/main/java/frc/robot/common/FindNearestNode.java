package frc.robot.common;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;

public class FindNearestNode {
    private Translation2d[] demoPoints;

    private Swerve swerve;

    public FindNearestNode(Swerve swerve) {
        this.swerve = swerve;

        demoPoints = new Translation2d[] { new Translation2d(15.1, 5.03), new Translation2d(15.03, 3.86) };
    }

    public Translation2d getTranslation() {
        Translation2d nearestNode = demoPoints[0];
        for (Translation2d translation : demoPoints) {
            if (DistanceToTranslation(translation) < DistanceToTranslation(nearestNode)) {
                nearestNode = translation;
            }
        }

        return nearestNode;
    }

    private double DistanceToTranslation(Translation2d translation) {
        return DistanceBetweenTranslations(swerve.getPose().getTranslation(), translation);
    }

    private double DistanceBetweenTranslations(Translation2d translationA, Translation2d translationB) {
        return Math.sqrt(Math.pow(translationA.getX() - translationB.getX(), 2)
                + Math.pow(translationA.getY() - translationB.getY(), 2));
    }
}
