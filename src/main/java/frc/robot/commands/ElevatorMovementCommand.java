package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.ElevatorConstants;

public class ElevatorMovementCommand extends Command {
  public static enum Direction {
    Up,
    Down
  }

  private final Direction direction;
  private static int i = 0;

  public ElevatorMovementCommand(Direction direction) {
    this.direction = direction;

    addRequirements(Subsystems.elevator);
  }

  @Override
  public void initialize() {
    switch (direction) {
      case Up:
        if (i < ElevatorConstants.EXTENSION_DISTS.size() - 1) {
          i++;
        }
        break;
      case Down:
        if (i > 0) {
          i--;
        }
        break;
    }
    Subsystems.elevator.setTargetHeight(ElevatorConstants.EXTENSION_DISTS.get(i));
  };

  @Override
  public boolean isFinished() {
    return true;
  }
}
