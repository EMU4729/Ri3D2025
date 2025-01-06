package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTPID {
  public final PIDController controller;
  private final NetworkTable table;

  private final DoubleEntry pidPEntry;
  private final DoubleEntry pidIEntry;
  private final DoubleEntry pidDEntry;

  public NTPID(PIDController controller, String name) {
    this.controller = controller;

    table = NetworkTableInstance.getDefault().getTable(name);

    pidPEntry = table.getDoubleTopic("pidP").getEntry(controller.getP());
    pidIEntry = table.getDoubleTopic("pidI").getEntry(controller.getI());
    pidDEntry = table.getDoubleTopic("pidD").getEntry(controller.getD());

    pidPEntry.set(controller.getP());
    pidIEntry.set(controller.getI());
    pidDEntry.set(controller.getD());
  }

  public void update() {
    controller.setPID(pidPEntry.get(), pidIEntry.get(), pidDEntry.get());
  }
}
