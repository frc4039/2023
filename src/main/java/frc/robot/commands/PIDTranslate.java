// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;

public class PIDTranslate extends CommandBase {
    private Swerve swerve;

    private DoubleSupplier xSup;
    private DoubleSupplier ySup;
    private DoubleSupplier rotSup;

    private PIDController rotationController = new PIDController(4.0, 0, 0);

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2);

    private PIDController xPidController = new PIDController(0.4, 0, 0);
    private PIDController yPidController = new PIDController(0.4, 0, 0);

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
        rotationController.reset();
        rotationController.setSetpoint(rotSup.getAsDouble());

        xPidController.setSetpoint(xSup.getAsDouble());
        yPidController.setSetpoint(ySup.getAsDouble());
    }

    @Override
    public void execute() {
        if (DriverStation.getAlliance().toString() == "Red") {
            double translationVal = -translationLimiter.calculate(
                    xPidController.calculate(swerve.getPose().getX()));
            double strafeVal = -strafeLimiter.calculate(
                    yPidController.calculate(swerve.getPose().getY()));

            /*
             * translationVal = -translationVal + Math.signum(-translationVal) *
             * VisionConstants.kTranslationFF;
             * strafeVal = -strafeVal + Math.signum(-strafeVal) * VisionConstants.kStrafeFF;
             * 
             * double rotationOutput =
             * rotationController.calculate(swerve.getYaw().getRadians());
             * 
             * double rotationVal = MathUtil.clamp(rotationOutput, -4, 4);
             * 
             * swerve.drive(new Translation2d(-translationVal,
             * -strafeVal).times(Constants.Swerve.kMaxSpeed), rotationVal,
             * true);
             */

        }
        if (DriverStation.getAlliance().toString() == "Blue") {
            double translationVal = translationLimiter.calculate(
                    xPidController.calculate(swerve.getPose().getX()));
            double strafeVal = strafeLimiter.calculate(
                    yPidController.calculate(swerve.getPose().getY()));

            /*
             * translationVal = translationVal + Math.signum(translationVal) *
             * VisionConstants.kTranslationFF;
             * strafeVal = strafeVal + Math.signum(strafeVal) * VisionConstants.kStrafeFF;
             * 
             * double rotationOutput =
             * rotationController.calculate(swerve.getYaw().getRadians());
             * 
             * double rotationVal = MathUtil.clamp(rotationOutput, -4, 4);
             * 
             * swerve.drive(new Translation2d(translationVal,
             * strafeVal).times(Constants.Swerve.kMaxSpeed), rotationVal,
             * true);
             */

        }
        double rotationOutput = rotationController.calculate(swerve.getYaw().getRadians());
        /*
         * double translationVal = translationLimiter.calculate(
         * xPidController.calculate(swerve.getPose().getX()));
         * double strafeVal = strafeLimiter.calculate(
         * yPidController.calculate(swerve.getPose().getY()));
         */
        double rotationVal = MathUtil.clamp(rotationOutput, -4, 4);

        translationVal = translationVal + Math.signum(translationVal) *
                VisionConstants.kTranslationFF;
        strafeVal = strafeVal + Math.signum(strafeVal) * VisionConstants.kStrafeFF;

        swerve.drive(new Translation2d(translationVal,
                strafeVal).times(Constants.Swerve.kMaxSpeed), rotationVal,
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
}
