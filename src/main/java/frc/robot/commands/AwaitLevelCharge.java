// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AwaitLevelCharge extends CommandBase {
    private final Swerve swerve;
    private final BooleanSupplier reverse;
    private boolean liftoff = false;

    public AwaitLevelCharge(Swerve swerve, BooleanSupplier reverse) {
        this.swerve = swerve;
        this.reverse = reverse;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!reverse.getAsBoolean() && swerve.getRawPitch() > 20.0
                || reverse.getAsBoolean() && swerve.getRawPitch() < -20.0) {
            liftoff = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (liftoff && (!reverse.getAsBoolean() && swerve.getRawPitch() < 15.0
                || reverse.getAsBoolean() && swerve.getRawPitch() > -15.0)) {
            return true;
        }

        return false;
    }
}