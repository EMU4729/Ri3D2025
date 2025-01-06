package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.PivotConstants;

public class PivotMovementCommand extends Command {
  public static enum Direction {
    TowardsReef,
    TowardsIntake
  }

  private final Direction direction;
  private static int i = 0;

  public PivotMovementCommand(Direction direction) {
    this.direction = direction;

    addRequirements(Subsystems.pivot);
  }

  @Override
  public void initialize() {
    switch (direction) {
      case TowardsReef:
        if (i < PivotConstants.PIVOT_ANGLES.size() - 1) {
          i++;
        }
        break;
      case TowardsIntake:
        if (i > 0) {
          i--;
        }
        break;
    }
    Subsystems.pivot.setTargetAngle(PivotConstants.PIVOT_ANGLES.get(i));
  };

  @Override
  public boolean isFinished() {
    return true;
  }
}
