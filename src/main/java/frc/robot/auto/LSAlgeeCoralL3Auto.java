package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.ActivateAlgaeL2;
import frc.robot.commands.ActivateAlgaeUnload;
import frc.robot.commands.ActivateCoralL3;
import frc.robot.commands.DriveAtAngle;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.TurnAndDriveToPose;
import frc.robot.commands.TurnToAngle;

public class LSAlgeeCoralL3Auto extends SequentialCommandGroup {
    public LSAlgeeCoralL3Auto(){
        addCommands(
            new InstantCommand(()->Subsystems.nav.setCurrentPose(new Pose2d(7.945, 4.955, Rotation2d.fromDegrees(180)))),
            new ParallelCommandGroup(
                new TurnAndDriveToPose(new Translation2d(5.51, 4.955), 0.5, 0.2, Rotation2d.fromDegrees(2)),
                new ActivateAlgaeL2(true)
            ),
            new TurnToAngle(Rotation2d.fromDegrees(240), Rotation2d.fromDegrees(2)),
            new DriveAtAngle(0.4, new Rotation2d(240), 1000),
            new DriveAtAngle(-0.4, new Rotation2d(240), 1000),
            new ActivateCoralL3(true),
            new TurnToAngle(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(2)),
            new DriveAtAngle(0.4, new Rotation2d(60), 1000),
            new DriveAtAngle(-0.4, new Rotation2d(60), 1000),
            new TurnAndDriveToPose(new Translation2d(9, 9.5), 0.5, 0.2, Rotation2d.fromDegrees(2)),
            new TurnAndDriveToPose(new Translation2d(9, 9), 0.5, 0.2, Rotation2d.fromDegrees(2)),
            new ActivateAlgaeUnload()
            

        );
    }
}
