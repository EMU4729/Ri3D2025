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

public class BasicDriveAuto extends SequentialCommandGroup {
    public BasicDriveAuto(){
        addCommands(
            new InstantCommand(()->Subsystems.nav.setCurrentPose(new Pose2d(7.945, 4.955, Rotation2d.fromDegrees(180)))),
            new DriveToPose(new Translation2d(5.51, 4.955), 0.5, 0.2),
            new DriveAtAngle(0, Rotation2d.fromDegrees(240), 2000)
        );
    }
}
