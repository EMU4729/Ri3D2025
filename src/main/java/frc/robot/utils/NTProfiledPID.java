package frc.robot.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTProfiledPID {
  public final ProfiledPIDController controller;
  private final NetworkTable table;

  private final DoubleEntry pidPEntry;
  private final DoubleEntry pidIEntry;
  private final DoubleEntry pidDEntry;
  private final DoubleEntry maxVEntry;
  private final DoubleEntry maxAEntry;

  public NTProfiledPID(ProfiledPIDController controller, String name) {
    this.controller = controller;

    table = NetworkTableInstance.getDefault().getTable(name);

    pidPEntry = table.getDoubleTopic("pidP").getEntry(controller.getP());
    pidIEntry = table.getDoubleTopic("pidI").getEntry(controller.getI());
    pidDEntry = table.getDoubleTopic("pidD").getEntry(controller.getD());
    maxVEntry = table.getDoubleTopic("maxV").getEntry(controller.getConstraints().maxVelocity);
    maxAEntry = table.getDoubleTopic("maxA").getEntry(controller.getConstraints().maxAcceleration);

    pidPEntry.set(controller.getP());
    pidIEntry.set(controller.getI());
    pidDEntry.set(controller.getD());
    maxVEntry.set(controller.getConstraints().maxVelocity);
    maxAEntry.set(controller.getConstraints().maxAcceleration);
  }

  public void update() {
    controller.setPID(pidPEntry.get(), pidIEntry.get(), pidDEntry.get());
    controller.setConstraints(new TrapezoidProfile.Constraints(maxVEntry.get(), maxAEntry.get()));
  }
}
