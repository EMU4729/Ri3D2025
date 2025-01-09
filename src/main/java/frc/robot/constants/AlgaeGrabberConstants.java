package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonMotorSupplier;

public class AlgaeGrabberConstants {
  protected AlgaeGrabberConstants() {
  }

  public static final MotorSupplier<WPI_TalonSRX> UPPER_MOTOR_ID = new TalonMotorSupplier(8).withBrake();
  public static final MotorSupplier<WPI_TalonSRX> LOWER_MOTOR_ID = new TalonMotorSupplier(9).withBrake().withInvert();
}
