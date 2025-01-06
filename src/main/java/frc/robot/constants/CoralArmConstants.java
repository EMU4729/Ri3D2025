package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.EncoderSupplier;
import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonMotorSupplier;

public class CoralArmConstants {
  protected CoralArmConstants() {
  }

  // note: none of these values are real
  public static final MotorSupplier<WPI_TalonSRX> MOTOR_ID = new TalonMotorSupplier(3).withBrake();
  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(new int[] { 0, 1 }, 0.002);
  public static final double[] ANGLES_DEGREES = new double[] { 0, 35, 180 };

  public static final double PID_P = 2;
  public static final double PID_I = 0;
  public static final double PID_D = 0;
}
