// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class PIDTranslate extends CommandBase {
    private Swerve swerve;

    private DoubleSupplier xSup;
    private DoubleSupplier ySup;
    private DoubleSupplier rotSup;

    private PIDController rotationController = new PIDController(4.0, 0, 0);

    private ProfiledPIDController xPidController = new ProfiledPIDController(AutoConstants.kPXController, 0, 0,
            AutoConstants.kPositionControllerConstraints);

    public PIDTranslate(Swerve swerve, DoubleSupplier xSup, DoubleSupplier ySup, DoubleSupplier rotSup) {
        this.swerve = swerve;
        this.xSup = xSup;
        this.ySup = ySup;
        this.rotSup = () -> Math.toRadians(rotSup.getAsDouble());

        rotationController.enableContinuousInput(0, 2 * Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {

        Translation2d deltaPosition = calculateGoalPosition();

        rotationController.reset();
        rotationController.setSetpoint(rotSup.getAsDouble());

        xPidController.reset(deltaPosition.getNorm(), -AutoConstants.kPositionControllerConstraints.maxVelocity);
        xPidController.setGoal(0.0);
    }

    @Override
    public void execute() {
        Translation2d deltaPosition = calculateGoalPosition();

        double velocity = xPidController.calculate(deltaPosition.getNorm());
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
        return false;
    }

    public Translation2d calculateGoalPosition() {
        Translation2d resultGoalPosition;

        Translation2d goalPos = new Translation2d(this.xSup.getAsDouble(), this.ySup.getAsDouble());
        Translation2d currentPos = swerve.getPose().getTranslation();
        Translation2d deltaPos = currentPos.minus(goalPos);

        double xOffset = MathUtil.clamp(Math.abs(deltaPos.getY()), 0, 0.5);

        if (goalPos.getX() < 4) {
            resultGoalPosition = deltaPos.minus(new Translation2d(xOffset, 0));
        } else {
            resultGoalPosition = deltaPos.plus(new Translation2d(xOffset, 0));
        }

        return resultGoalPosition;
    }
}
