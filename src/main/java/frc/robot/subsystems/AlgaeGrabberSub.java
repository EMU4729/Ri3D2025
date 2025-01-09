package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeGrabberConstants;

public class AlgaeGrabberSub extends SubsystemBase {
  private final WPI_TalonSRX upperMotor = AlgaeGrabberConstants.UPPER_MOTOR_ID.get();
  private final WPI_TalonSRX lowerMotor = AlgaeGrabberConstants.LOWER_MOTOR_ID.get();

  public AlgaeGrabberSub() {
    // we're assuming one of lowerMotor or upperMotor is inverted so that the
    // rollers are either both spinning inwards or both spinning outwards
    lowerMotor.follow(upperMotor);
  }

  public void set(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    upperMotor.set(speed);
  }
  public void loadSlow(){
    set(0.2);
  }
  public void loadfast(){
    set(0.4);
  }
  public void stop(){
    set(0);
  }
  public void unload(){
    set(-1);
  }
}
