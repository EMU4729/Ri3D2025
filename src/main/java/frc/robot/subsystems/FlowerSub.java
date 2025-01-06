package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FlowerConstants;

public class FlowerSub extends SubsystemBase {
  private final DoubleSolenoid solenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      FlowerConstants.SOLENOID_FORWARD_ID,
      FlowerConstants.SOLENOID_REVERSE_ID);

  public FlowerSub() {
    retract();
  }

  public void extend() {
    solenoid.set(Value.kForward);
  }

  public void retract() {
    solenoid.set(Value.kReverse);
  }
}
