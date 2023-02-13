package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionHub extends SubsystemBase {
    private final NetworkTable nt;
    private DoublePublisher[] pubs;
    private final DoublePublisher totalCurrentPub;
    private final PowerDistribution m_powerDistributionHub;

    public PowerDistributionHub() {
        nt = NetworkTableInstance.getDefault().getTable("pdhCurrent");
        m_powerDistributionHub = new PowerDistribution(5, ModuleType.kRev);

        totalCurrentPub = nt.getDoubleTopic("Total Current").publish();
        pubs = new DoublePublisher[m_powerDistributionHub.getNumChannels()];
        for (int i = 0; i < pubs.length; i++) {
            pubs[i] = nt.getDoubleTopic(Integer.toString(i)).publish();
        }
    }

    public double getCurrent(int channel) {
        return m_powerDistributionHub.getCurrent(channel);
    }

    public double getTotalCurrent() {
        return m_powerDistributionHub.getTotalCurrent();
    }

    @Override
    public void periodic() {
        totalCurrentPub.set(m_powerDistributionHub.getTotalCurrent());
        for (int i = 0; i < m_powerDistributionHub.getNumChannels(); i++) {
            pubs[i].set(m_powerDistributionHub.getCurrent(i));
        }
    }
}
