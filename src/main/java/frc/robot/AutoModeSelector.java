package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.DropAndDrive;
import frc.robot.autos.DropAndDriveYellowSide;

public class AutoModeSelector {
	private SendableChooser<Command> autoModeChooser = new SendableChooser<>();

	public AutoModeSelector(RobotContainer container){
		autoModeChooser.setDefaultOption("Select...", doNothingCommand(container));
		// autoModeChooser.addOption("Drop and drive", dropAndDrive(container));
		autoModeChooser.addOption("Drop and drive (yellow side)", dropAndDriveYellow(container));
	}

	public SendableChooser<Command> getAutoChooser(){
		return autoModeChooser;
	}

	private SequentialCommandGroup doNothingCommand(RobotContainer container){
		SequentialCommandGroup command = new SequentialCommandGroup();
		container.getSwerve().resetPoseAndGyro();
		return command;
	}

	private SequentialCommandGroup dropAndDrive(RobotContainer container){
		return new DropAndDrive(container);
	}

	private SequentialCommandGroup dropAndDriveYellow(RobotContainer container){
		return new DropAndDriveYellowSide(container);
	}

}
