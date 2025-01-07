package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems;
import frc.robot.commands.DriveAtAngle;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.TurnAndDriveToPose;
import frc.robot.commands.TurnToAngle;

public class BasicDriveAuto extends SequentialCommandGroup {
    public BasicDriveAuto() {
        addCommands(
                new InstantCommand(
                        () -> Subsystems.nav.setCurrentPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
                //new TurnAndDriveToPose(new Translation2d(5.51, 4.955), 0.5, 0.2, Rotation2d.fromDegrees(2)),
                //new TurnToAngle(Rotation2d.fromDegrees(240), Rotation2d.fromDegrees(2))
                new DriveToPose(new Translation2d(1, 0), 0.4, 0.2, false)
            );
    }
}
