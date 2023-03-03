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
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerveAtFixedRotation extends CommandBase {
	private Swerve m_swerve;

  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
	private double rotation;

	private PIDController rotationController = new PIDController(4.0, 0, 0);

	private SlewRateLimiter translationLimiter = new SlewRateLimiter(2);
	private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2);

	public TeleopSwerveAtFixedRotation(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, double rotation) {
		this.m_swerve = swerve;
		this.translationSup = translationSup;
		this.strafeSup = strafeSup;
		this.rotation = Math.toRadians(rotation);

		rotationController.enableContinuousInput(0, 2 * Math.PI);

		addRequirements(m_swerve);
	}

	@Override
	public void initialize() {
		rotationController.reset();
		rotationController.setSetpoint(rotation);
	}

	@Override
	public void execute() {
		double rotationOutput = rotationController.calculate(m_swerve.getYaw().getRadians());

		double translationVal = translationLimiter.calculate(
			MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.translationStickDeadband));
		double strafeVal = strafeLimiter.calculate(
			MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.translationStickDeadband));
		double rotationVal = MathUtil.clamp(rotationOutput, -4, 4);

		m_swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal, true);
	}

	@Override
	public void end(boolean interrupted) {
    m_swerve.drive(new Translation2d(0, 0), 0, false);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
