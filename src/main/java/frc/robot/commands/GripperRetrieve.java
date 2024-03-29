// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class GripperRetrieve extends CommandBase {
    private final Gripper m_Gripper;

    public GripperRetrieve(Gripper gripper) {
        m_Gripper = gripper;
        addRequirements(m_Gripper);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_Gripper.setClose();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Gripper.setClose();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_Gripper.setOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if (m_Gripper.getState() == Value.kForward) {
            return true;
        } else {
            return false;
        }

    }
}
