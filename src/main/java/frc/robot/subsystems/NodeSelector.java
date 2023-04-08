package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.common.Nodes;

public class NodeSelector extends SubsystemBase {

    private static Nodes nodes;

    private boolean closestNodeSelected;
    private Translation2d selectedNodeTranslation;
    private String selectedNodeLabel;
    private int selectedNodeNumber;
    private Alliance alliance;
    private NetworkTable table;
    private int lastNodeNT = -1;
    private int lastHeightNT = -1;
    private IntegerEntry selectedNodeEntry;
    private IntegerEntry selectedHeightEntry;

    public NodeSelector(RobotContainer container) {

        nodes = new Nodes(container.getSwerve());

        closestNodeSelected = true;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Node Selector Data");

        selectedNodeEntry = table.getIntegerTopic("selectedNode")
                .getEntry(0);
        selectedHeightEntry.set(0);
    }

    public Translation2d getSelectedNodeTranslation() {
        return selectedNodeTranslation;
    }

    public String getSelectedNodeLabel() {
        return selectedNodeLabel;
    }

    public void increaseSelectedNode() {
        closestNodeSelected = false;
        if (closestNodeSelected
                || !closestNodeSelected && selectedNodeNumber == nodes.getTranslations(Alliance.Red).length - 1) {
            selectedNodeNumber = 0;
        } else {
            selectedNodeNumber++;
        }
    }

    public void decreaseSelectedNode() {
        closestNodeSelected = false;
        if (closestNodeSelected || !closestNodeSelected && selectedNodeNumber == 0) {
            selectedNodeNumber = nodes.getTranslations(Alliance.Red).length - 1;
        } else {
            selectedNodeNumber--;
        }
    }

    public void selectNode(int node) {
        closestNodeSelected = false;
        selectedNodeNumber = node - 1;
    }

    public void selectClosestNode() {
        closestNodeSelected = true;
    }

    @Override

    public void periodic() {
        alliance = DriverStation.getAlliance();

        if (selectedNodeEntry.get() + 1 != lastNodeNT || selectedHeightEntry.get() + 1 != lastHeightNT) {
            lastNodeNT = (int) selectedNodeEntry.get() + 1;
            lastHeightNT = (int) selectedHeightEntry.get();
            closestNodeSelected = false;
            selectNode((int) selectedNodeEntry.get() + 1);
            selectedNodeTranslation = nodes.getTranslations(alliance)[selectedNodeNumber];
            selectedNodeLabel = nodes.getTranslationLabel(selectedNodeTranslation, alliance);
            // TODO: Pass scoring height to automatic scoring command, once created.
        } else {
            if (closestNodeSelected) {
                selectedNodeTranslation = nodes.getNearestTranslation(alliance);
                selectedNodeLabel = "Closest Node";
            } else {
                selectedNodeTranslation = nodes.getTranslations(alliance)[selectedNodeNumber];
                selectedNodeLabel = nodes.getTranslationLabel(selectedNodeTranslation, alliance);
            }
        }

    }
}
