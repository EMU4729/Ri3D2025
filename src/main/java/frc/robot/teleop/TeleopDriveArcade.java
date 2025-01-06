package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.RangeMath.DriveBaseFit;

/**
 * The Arcade Teleop command
 */
public class TeleopDriveArcade extends Command {
  private final DriveBaseFit settings;

  public TeleopDriveArcade() {
    this(DriveConstants.PILOT_SETTINGS);
  }

  public TeleopDriveArcade(DriveBaseFit settings) {
    this.settings = settings;
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    double[] control = settings.fitTank(
        OI.pilot.getLeftY(),
        OI.pilot.getRightX(),
        OI.pilot.rightBumper().getAsBoolean() ? 1 : 0,
        OI.pilot.getRightTriggerAxis());
    double throttle = control[0];
    double steering = control[2];

    // Invert steering when throttle >= 0 to mimic car controls
    // if (throttle > 0) {
    // steering *= -1;
    // }

    // flips the direction of forward based on controller button
    if (Variables.invertDriveDirection) {
      throttle *= -1;
    }

    Subsystems.drive.arcade(throttle, steering);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
