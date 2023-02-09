package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Intake;

public class IntakeSpin extends CommandBase {
    private final Intake m_intake;

    public IntakeSpin(Intake m_intake) {
        this.m_intake = m_intake;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.setSpinningMotorOn();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setSpinningMotorOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
