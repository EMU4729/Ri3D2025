package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.DriveAtAngle;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.TurnAndDriveToPose;
import frc.robot.commands.TurnToAngle;

public class DriveBox extends SequentialCommandGroup {
    public DriveBox(double dist) {
        addCommands(
            new InstantCommand(
                    () -> Subsystems.nav.setCurrentPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
            new DriveToPose(new Translation2d(dist, 0), 0.8, 0.75, 0.05, false),
            new TurnToAngle(Rotation2d.kCCW_90deg, 0.8, Rotation2d.fromDegrees(2), false),
            new DriveToPose(new Translation2d(dist, dist), 0.8, 0.75, 0.05, false),
            new TurnToAngle(Rotation2d.k180deg, 0.8, Rotation2d.fromDegrees(2), false),
            new DriveToPose(new Translation2d(0, dist), 0.8, 0.75, 0.05, false),
            new TurnToAngle(Rotation2d.kCW_90deg, 0.8, Rotation2d.fromDegrees(2), false),
            new DriveToPose(new Translation2d(0, 0), 0.8, 0.75, 0.05, false),
            new TurnToAngle(Rotation2d.kZero, 0.8, Rotation2d.fromDegrees(2), false)

        );
    }
    
}
