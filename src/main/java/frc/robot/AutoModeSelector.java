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
        autoModeChooser.addOption("1 Piece Mobility Balance (any preload)", dropMobilityAndBalance(container));
        autoModeChooser.addOption("1.5 Balance (Yellow Preload Only)", oneHalfBalance(container));
        // autoModeChooser.addOption("Drop, drive, strafe, and pickup", new
        // DropAndDriveAndStrafeAndPickup(container));
        // autoModeChooser.addOption("Middle Pickup (red)",
        // middlePickupStraight(container, true));
        // autoModeChooser.addOption("Middle Pickup (blue)",
        // middlePickupStraight(container, false));
        // autoModeChooser.addOption("Middle Pickup Balance",
        // middlePickupBalance(container));

        autoModeChooser.addOption("Flat Side 2 Purple (Red)", twoPiecePurpleBarrier(container, true));
        autoModeChooser.addOption("Flat Side 2 Purple (Blue)", twoPiecePurpleBarrier(container, false));
        autoModeChooser.addOption("Bump Side 2 Purple (Red)", bumpSideTwoPurple(container, true));
        autoModeChooser.addOption("Bump Side 2 Purple (Blue)", bumpSideTwoPurple(container, false));
        autoModeChooser.addOption("Bump Side 3 Purple (Red)", bumpSideThreePurple(container, true));
        autoModeChooser.addOption("Bump Side 3 Purple (Blue)", bumpSideThreePurple(container, false));
        autoModeChooser.addOption("Bump side 1 Yellow + Mobility", new DropMobilityChargeSide(container));
        autoModeChooser.addOption("Flat Side Yellow, Purple (Red)", yellowPurpleFlatSide(container, true));
        autoModeChooser.addOption("Flat Side Yellow, Purple (Blue)", yellowPurpleFlatSide(container, false));
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

    private SequentialCommandGroup middlePickupStraight(RobotContainer container, boolean isRed) {
        return new MiddlePickupStraight(container, isRed);
    }

    private SequentialCommandGroup middlePickupBalance(RobotContainer container) {
        return new MiddlePickupBalance(container);
    }

    private SequentialCommandGroup twoPiecePurpleBarrier(RobotContainer container, boolean isRed) {
        return new TwoPiecePurpleBarrier(container, isRed);
    }

    private SequentialCommandGroup oneHalfBalance(RobotContainer container) {
        return new OneHalfBalance(container);
    }

    private SequentialCommandGroup bumpSideTwoPurple(RobotContainer container, boolean isRed) {
        return new BumpSideTwoPurple(container, isRed);
    }

    private SequentialCommandGroup bumpSideThreePurple(RobotContainer container, boolean isRed) {
        return new BumpSideThreePurple(container, isRed);
    }

    private SequentialCommandGroup yellowPurpleFlatSide(RobotContainer container, boolean isRed) {
        return new YellowPurpleFlatSide(container, isRed);
    }

}
