// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class GamePieceSelector extends SubsystemBase {

    public enum Gamepiece {
        YELLOW,
        PURPLE
    }

    public Gamepiece currentGamepieceState;

    /** Creates a new GamePieceSelector. */
    public GamePieceSelector(RobotContainer container) {

        currentGamepieceState = Gamepiece.YELLOW;

    }

    public void setGamepieceYellow() {
        currentGamepieceState = Gamepiece.YELLOW;
    }

    public void setGamepiecePurple() {
        currentGamepieceState = Gamepiece.PURPLE;
    }

    public Gamepiece getCurrentGamepiece() {
        return currentGamepieceState;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
