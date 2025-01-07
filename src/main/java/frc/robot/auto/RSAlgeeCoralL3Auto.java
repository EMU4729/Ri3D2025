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
import frc.robot.commands.ActivateStow;
import frc.robot.commands.DriveAtAngle;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.TurnAndDriveToPose;
import frc.robot.commands.TurnToAngle;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.AutoConstants;

public class RSAlgeeCoralL3Auto extends SequentialCommandGroup {
    public RSAlgeeCoralL3Auto(){
        addCommands(
            new InstantCommand(
                ()->Subsystems.nav.setCurrentPose(
                        AutoConstants.AutoPoints.byAlliance(AutoConstants.AutoPoints.START_RIGHT))),
            new WaitCommand(0.2),
            new InstantCommand(
                ()->Subsystems.nav.setCurrentPose(
                        AutoConstants.AutoPoints.byAlliance(AutoConstants.AutoPoints.START_RIGHT))),
            new ParallelCommandGroup(
                new TurnAndDriveToPose(new Translation2d(5.705, AutoConstants.AutoPoints.START_RIGHT.getY()), 
                                    0.5, 0.2, Rotation2d.fromDegrees(3)),
                new ActivateAlgaeL2(true)
            ),
            new TurnToAngle(Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(3)),
            new DriveToPose(AutoConstants.AutoPoints.REEF_FAR_RIGHT, 0.5, 0.1),
            new DriveAtAngle(-0.5, Rotation2d.fromDegrees(120), 1000),
            new InstantCommand(()->Subsystems.algaeArm.setTargetAngle(AlgaeArmConstants.ANGLES.STOW)),
            new ActivateCoralL3(true),
            new TurnToAngle(Rotation2d.fromDegrees(300), Rotation2d.fromDegrees(3)),
            new DriveAtAngle(-0.4, Rotation2d.fromDegrees(300), 1500),
            new DriveAtAngle(0.4, Rotation2d.fromDegrees(300), 1000),
            new TurnAndDriveToPose(new Translation2d(AutoConstants.AutoPoints.PROCESSOR.getX(), 
                                                     AutoConstants.AutoPoints.PROCESSOR.getY()+0.5), 
                                                     0.5, 0.2, Rotation2d.fromDegrees(3)),
            new TurnAndDriveToPose(AutoConstants.AutoPoints.PROCESSOR, 0.5, 0.2, Rotation2d.fromDegrees(3)),
            new ActivateAlgaeUnload(),
            new WaitCommand(2),
            new ActivateStow()
        );
    }
}
