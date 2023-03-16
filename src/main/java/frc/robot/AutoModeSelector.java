package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.*;

public class AutoModeSelector {
    private SendableChooser<Command> autoModeChooser = new SendableChooser<>();

    public AutoModeSelector(RobotContainer container) {
        autoModeChooser.setDefaultOption("Select...", doNothingCommand(container));
        // autoModeChooser.addOption("Drop and drive (yellow side)",
        // dropAndDriveYellow(container));
        autoModeChooser.addOption("Drop and balance", dropAndBalanceYellow(container));
        autoModeChooser.addOption("Drop, Mobility, and balance", dropMobilityAndBalance(container));
        // autoModeChooser.addOption("Drop, drive, strafe, and pickup", new
        // DropAndDriveAndStrafeAndPickup(container));
        autoModeChooser.addOption("Middle Pickup", middlePickupStraight(container));
        // autoModeChooser.addOption("Middle Pickup Balance",
        // middlePickupBalance(container));
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoModeChooser;
    }

    private SequentialCommandGroup doNothingCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();
        container.getSwerve().resetPoseAndGyro();
        return command;
    }

    private SequentialCommandGroup dropAndDriveYellow(RobotContainer container) {
        return new DropAndDriveYellowSide(container);
    }

    private SequentialCommandGroup dropAndBalanceYellow(RobotContainer container) {
        return new DropAndBalanceYellowSide(container);
    }

    private SequentialCommandGroup dropMobilityAndBalance(RobotContainer container) {
        return new DropMobilityBalanceAuto(container);
    }

    private SequentialCommandGroup middlePickupStraight(RobotContainer container) {
        return new MiddlePickupStraight(container);
    }

    private SequentialCommandGroup middlePickupBalance(RobotContainer container) {
        return new MiddlePickupBalance(container);
    }

}
