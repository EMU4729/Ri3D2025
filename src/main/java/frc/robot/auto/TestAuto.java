package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto() {
    addCommands(
        Subsystems.drive.runOnce(() -> Subsystems.drive.arcade(1, 0)),
        new WaitCommand(1),
        Subsystems.drive.runOnce(Subsystems.drive::off));
  }
}
