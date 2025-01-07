package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.AutoConstants.AutoPoints;

public class TurnAndDriveToPose extends SequentialCommandGroup{
    public TurnAndDriveToPose(Translation2d targetPose, double targetSpeed, double driveTollerance, Rotation2d turnTollerance){
        this(targetPose, targetSpeed, driveTollerance, turnTollerance, true);
    }
    public TurnAndDriveToPose(Translation2d targetPose, double targetSpeed, double driveTollerance, Rotation2d turnTollerance, boolean byAlliance){
        addCommands(
            new TurnToPose(targetPose, turnTollerance),
            new DriveToPose(targetPose, targetSpeed, driveTollerance)
        );
    }
}
