package frc.robot.utils;

import frc.robot.RobotContainer;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.ConstantesShuffleboard;

public class DriverReadouts {

    private Field2d field;

    public DriverReadouts(RobotContainer container) {
        ConstantesShuffleboard.AutonomousTab.add("Autonomous Mode", PathChooser.getModeChooser()).withSize(4, 1).withPosition(0, 0);

        field = new Field2d();
        ConstantesShuffleboard.AutonomousTab.add("Field", field).withPosition(4, 0).withSize(6, 4);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        }); 
    }
}