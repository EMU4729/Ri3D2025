package frc.robot.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.EncoderSupplier;
import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonMotorSupplier;

public class CoralArmConstants {
  protected CoralArmConstants() {
  }

  public static class ANGLES {
    public static Angle STOW = Degree.of(90);
    public static Angle LOAD = Degree.of(35);
    public static Angle UNLOAD = Degree.of(-35);
  }

  // note: none of these values are real
  public static final MotorSupplier<WPI_TalonSRX> MOTOR_ID = new TalonMotorSupplier(8).withBrake();
  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(new int[] { 4, 5 }, 0.002);
  //public static final double[] ANGLES_DEGREES = new double[] { 0, 35, 180 };

  public static final double PID_P = 2;
  public static final double PID_I = 0;
  public static final double PID_D = 0;
}
