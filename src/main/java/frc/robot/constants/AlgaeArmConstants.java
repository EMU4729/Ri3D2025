package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;

public class AlgaeArmConstants {
  protected AlgaeArmConstants() {
  }

  public static final int MOTOR_ID = 3;

  // public static final List<Angle> PIVOT_ANGLES = List.of(
  // Degrees.of(0), Degrees.of(35), Degrees.of(200), Degrees.of(220));

  public static class ANGLES {
    public static Angle STOW = Degrees.of(0);
    public static Angle LOAD_START = Degrees.of(35);
    public static Angle LOAD_END = Degrees.of(45);
    public static Angle UNLOAD = Degrees.of(-35);
  }

  public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

  public static final double PID_P = 0.1;
  public static final double PID_I = 0;
  public static final double PID_D = 0;
  public static final double PID_FF = 0;

  // sim constants
  public static final DCMotor GEARBOX = DCMotor.getFalcon500(1);
  public static final double GEARING_RATIO = 1;
  public static final double ARM_MASS_KG = 1;
  public static final double ARM_LENGTH_METERS = 1;
  public static final Angle ARM_MIN_ANGLE = ANGLES.STOW;
  public static final Angle ARM_MAX_ANGLE = ANGLES.UNLOAD;
}
