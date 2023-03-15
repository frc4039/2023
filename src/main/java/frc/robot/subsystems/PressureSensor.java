package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PressureSensor {
    static Compressor analogCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

    public static double getAnalogPressureReading() {
        return analogCompressor.getPressure();
    }
}