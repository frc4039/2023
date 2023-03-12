// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AwaitLevelCharge extends CommandBase {
    private final Swerve swerve;
    private boolean liftoff = false;

    public AwaitLevelCharge(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (Math.abs(swerve.getRawPitch()) > 20.0) {
            liftoff = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (liftoff && Math.abs(swerve.getRawPitch()) < 12.0) {
            return true;
        }

        return false;
    }
}