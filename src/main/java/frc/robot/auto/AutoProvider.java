package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtAngle;
import frc.robot.commands.DriveToPose;

/**
 * Provides the default command for autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final SendableChooser<Command> chooser;

  private AutoProvider() {
    chooser = new SendableChooser<>(); // pub for shuffle board
    // chooser = AutoBuilder.buildAutoChooser();
    // chooser.setDefaultOption("disabled", new InstantCommand(() -> {
    // }, Subsystems.swerveDrive));
    chooser.addOption("test", new TestAuto());
    chooser.addOption("test2", new SequentialCommandGroup(new DriveAtAngle(0.4, Rotation2d.fromDegrees(45), 4000), 
                                                               new DriveToPose(new Translation2d(10, 5), 0.7, 0.1)));
    SmartDashboard.putData("Auto Chooser", chooser);
  }

  public static AutoProvider getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new AutoProvider());
    }
    return inst.get();
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}