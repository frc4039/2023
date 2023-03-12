package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.common.Nodes;

public class NodeSelector extends SubsystemBase {

    private static Nodes nodes;

    private boolean closestNodeSelected;
    private Translation2d selectedNodeTranslation;
    private String selectedNodeLabel;
    private int selectedNodeNumber;

    public NodeSelector(RobotContainer container) {

        nodes = new Nodes(container.getSwerve());

        closestNodeSelected = true;
        selectedNodeTranslation = nodes.getNearestTranslation();
        selectedNodeLabel = nodes.getTranslationLabel(selectedNodeTranslation);
    }

    public Translation2d getSelectedNodeTranslation() {
        return selectedNodeTranslation;
    }

    public String getSelectedNodeLabel() {
        return selectedNodeLabel;
    }

    public void increaseSelectedNode() {
        closestNodeSelected = false;
        if (closestNodeSelected || !closestNodeSelected && selectedNodeNumber == nodes.getTranslations().length - 1) {
            selectedNodeNumber = 0;
        } else {
            selectedNodeNumber++;
        }
    }

    public void decreaseSelectedNode() {
        closestNodeSelected = false;
        if (closestNodeSelected || !closestNodeSelected && selectedNodeNumber == 0) {
            selectedNodeNumber = nodes.getTranslations().length - 1;
        } else {
            selectedNodeNumber--;
        }
    }

    public void selectClosestNode() {
        closestNodeSelected = true;
    }

    @Override
    public void periodic() {
        if (closestNodeSelected) {
            selectedNodeTranslation = nodes.getNearestTranslation();
            selectedNodeLabel = "Closest Node";
        } else {
            selectedNodeTranslation = nodes.getTranslations()[selectedNodeNumber];
            selectedNodeLabel = nodes.getTranslationLabel(selectedNodeTranslation);
        }

    }
}
