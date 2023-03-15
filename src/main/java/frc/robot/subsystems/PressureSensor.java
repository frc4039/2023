package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PressureSensor extends SubsystemBase {
    static Compressor analogCompressor = new Compressor(7, PneumaticsModuleType.REVPH); // This uses CANIDs, not
                                                                                        // Pneumatic Hub IDs...

    public static double getAnalogPressureReading() {
        return analogCompressor.getPressure();
    }
}