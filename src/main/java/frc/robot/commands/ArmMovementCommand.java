package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.ArmConstants;

public class ArmMovementCommand extends Command {
  public static enum Direction {
    TowardsReef,
    TowardsIntake
  }

  private final Direction direction;
  private static int i = 0;

  public ArmMovementCommand(Direction direction) {
    this.direction = direction;

    addRequirements(Subsystems.arm);
  }

  @Override
  public void initialize() {
    switch (direction) {
      case TowardsReef:
        if (i < ArmConstants.PIVOT_ANGLES.size() - 1) {
          i++;
        }
        break;
      case TowardsIntake:
        if (i > 0) {
          i--;
        }
        break;
    }
    Subsystems.arm.setTargetAngle(ArmConstants.PIVOT_ANGLES.get(i));
  };

  @Override
  public boolean isFinished() {
    return true;
  }
}
