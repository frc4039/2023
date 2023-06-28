// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.BlinkinConstants;

/** Add your docs here. */
public class BlinkinGamePiece extends SubsystemBase {
    private Spark m_BlinkinStrip;

    public BlinkinGamePiece() {
        m_BlinkinStrip = new Spark(BlinkinConstants.kBlinkinPWMPort);
        SetRainbow();
    }

    public void SetColour(double colourValue) {
        if ((colourValue >= -1.0) && (colourValue <= 1.0)) {
            m_BlinkinStrip.set(colourValue);
        }
    }

    public void BlinkAndStopColour() {
        double currentColourValue = m_BlinkinStrip.get();
        SetColour(BlinkinConstants.kColourValueOff);
        SetColour(currentColourValue);
        SetColour(BlinkinConstants.kColourValueOff);
        SetColour(currentColourValue);
    }

    public void SetRainbow() {
        SetColour(BlinkinConstants.kColourValueRainbow);
    }
}
