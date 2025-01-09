package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.EncoderSupplier;
import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonMotorSupplier;

public class CoralArmConstants {
  protected CoralArmConstants() {
  }

  public static class ANGLES {
    public static Angle STOW = Degrees.of(0);
    public static Angle LOAD = Degrees.of(35);
    public static Angle UNLOAD = Degrees.of(-35);
  }

  // note: none of these values are real
  public static final MotorSupplier<WPI_TalonSRX> MOTOR_ID = new TalonMotorSupplier(7).withBrake();
  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(new int[] { 6, 7 }, 0.002);
  //public static final double[] ANGLES_DEGREES = new double[] { 0, 35, 180 };

  public static final double PID_P = 2;
  public static final double PID_I = 0;
  public static final double PID_D = 0;
  public static final double PID_FF = 0;
public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
}
