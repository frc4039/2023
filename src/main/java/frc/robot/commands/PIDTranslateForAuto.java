// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class PIDTranslateForAuto extends CommandBase {
    private Swerve swerve;

    private double xSup;
    private double ySup;
    private double rotSup;

    private OffsetNeeded needsOffset;
    private boolean useVision;

    private PIDController rotationController = new PIDController(4.0, 0, 0);

    private final ProfiledPIDController kXPidControllerDefault = new ProfiledPIDController(AutoConstants.kPXController,
            0, 0,
            AutoConstants.kPositionControllerConstraints);

    private ProfiledPIDController _xPidController;

    public PIDTranslateForAuto(Swerve swerve, double xSup, double ySup, double rotSup, OffsetNeeded needsOffset,
            boolean useVision) {
        this.swerve = swerve;
        this.xSup = xSup;
        this.ySup = ySup;
        this.rotSup = Math.toRadians(rotSup);
        this.needsOffset = needsOffset;
        this.useVision = useVision;

        rotationController.enableContinuousInput(0, 2 * Math.PI);

        _xPidController = kXPidControllerDefault;

        addRequirements(swerve);
    }

    public PIDTranslateForAuto(Swerve swerve, Pose2d pose, OffsetNeeded needsOffset, boolean useVision) {
        this.swerve = swerve;
        this.xSup = pose.getX();
        this.ySup = pose.getY();
        this.rotSup = pose.getRotation().getRadians();
        this.needsOffset = needsOffset;
        this.useVision = useVision;

        rotationController.enableContinuousInput(0, 2 * Math.PI);

        _xPidController = kXPidControllerDefault;

        addRequirements(swerve);
    }

    public PIDTranslateForAuto(Swerve swerve, Pose2d pose, OffsetNeeded needsOffset,
            Constraints constraints, boolean useVision) {
        this.swerve = swerve;
        this.xSup = pose.getX();
        this.ySup = pose.getY();
        this.rotSup = pose.getRotation().getRadians();
        this.needsOffset = needsOffset;
        this.useVision = useVision;

        rotationController.enableContinuousInput(0, 2 * Math.PI);

        _xPidController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0, constraints);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {

        Translation2d deltaPosition = calculateGoalPosition();

        rotationController.reset();
        rotationController.setSetpoint(rotSup);

        _xPidController.reset(deltaPosition.getNorm());
        _xPidController.setGoal(0.0);
    }

    @Override
    public void execute() {
        Translation2d deltaPosition = calculateGoalPosition();

        if (useVision) {
            if (!swerve.isUsingAutoVision() && deltaPosition.getNorm() < 1.2) {
                swerve.enableAutoVisionTracking();
            }
        } else {
            if (swerve.isUsingAutoVision()) {
                swerve.disableAutoVisionTracking();
            }
        }

        double velocity = _xPidController.calculate(deltaPosition.getNorm());
        Translation2d directionUnitVector = deltaPosition.div(deltaPosition.getNorm());
        Translation2d output = directionUnitVector.times(velocity);

        double rotationOutput = rotationController.calculate(swerve.getYaw().getRadians());
        double rotationVal = MathUtil.clamp(rotationOutput, -4, 4);

        swerve.autoDrive(output.times(Constants.Swerve.kMaxSpeed), rotationVal,
                true);

    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false);
    }

    @Override
    public boolean isFinished() {
        return calculateGoalPosition().getNorm() < 0.1;
    }

    public Translation2d calculateGoalPosition() {
        Translation2d resultGoalPosition;

        Translation2d goalPos = new Translation2d(this.xSup, this.ySup);
        Translation2d currentPos = swerve.getPose().getTranslation();
        Translation2d deltaPos = currentPos.minus(goalPos);

        if (needsOffset == OffsetNeeded.XMinus) {

            double yOffset = MathUtil.clamp(Math.abs(deltaPos.getX()), 0, 2.5);
            resultGoalPosition = deltaPos.minus(new Translation2d(0, yOffset));

            return resultGoalPosition;
        }
        if (needsOffset == OffsetNeeded.XPlus) {

            double yOffset = MathUtil.clamp(Math.abs(deltaPos.getX()), 0, 2.5);
            resultGoalPosition = deltaPos.plus(new Translation2d(0, yOffset));

            return resultGoalPosition;
        } else if (needsOffset == OffsetNeeded.XPlusSmaller) {

            double yOffset = MathUtil.clamp(Math.abs(deltaPos.getX()), 0, 0.6);
            resultGoalPosition = deltaPos.plus(new Translation2d(0, yOffset));

            return resultGoalPosition;

        } else if (needsOffset == OffsetNeeded.Y) {
            double xOffset = MathUtil.clamp(Math.abs(deltaPos.getY()), 0, 0.5);

            if (goalPos.getX() < 4) {
                resultGoalPosition = deltaPos.minus(new Translation2d(xOffset, 0));
            } else {
                resultGoalPosition = deltaPos.plus(new Translation2d(xOffset, 0));
            }

            return resultGoalPosition;
        } else {
            return deltaPos;
        }
    }

    public enum OffsetNeeded {
        None,
        XMinus,
        XPlus,
        XPlusSmaller,
        Y
    }
}
