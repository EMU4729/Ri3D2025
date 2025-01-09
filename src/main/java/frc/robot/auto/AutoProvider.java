package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveAtAngle;

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
    chooser.addOption("box", new DriveBox( 1));
    chooser.addOption("drive0.3", new BasicDriveAuto(0.3));
    chooser.addOption("drive1", new BasicDriveAuto(1));
    chooser.addOption("drive2", new BasicDriveAuto(2));
    chooser.addOption("drive5", new BasicDriveAuto(5));
    chooser.addOption("turn180", new BasicTurnAuto(Rotation2d.k180deg));
    chooser.addOption("turn90ccw", new BasicTurnAuto(Rotation2d.kCCW_90deg));
    chooser.addOption("turn90cw", new BasicTurnAuto(Rotation2d.kCW_90deg));
    chooser.addOption("test2", new SequentialCommandGroup(new DriveAtAngle(0.4, 0.5, Rotation2d.fromDegrees(45), 10000)));
    chooser.addOption("left Algae Coral L3", new LSAlgeeCoralL3Auto());
    chooser.addOption("right Algae Coral L3", new RSAlgeeCoralL3Auto());
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