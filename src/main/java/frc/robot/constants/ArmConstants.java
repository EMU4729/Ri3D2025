package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;

public class ArmConstants {
  protected ArmConstants() {
  }

  public static final int MOTOR_ID = 3;

  public static final List<Angle> PIVOT_ANGLES = List.of(
      Degrees.of(0), Degrees.of(35), Degrees.of(200), Degrees.of(220));

  public static final InvertedValue MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;

  public static final double PID_P = 2;
  public static final double PID_I = 0;
  public static final double PID_D = 0;
  public static final double PID_FF = 0;

  // sim constants
  public static final DCMotor GEARBOX = DCMotor.getFalcon500(1);
  public static final double GEARING_RATIO = 1;
  public static final double ARM_MASS_KG = 1;
  public static final double ARM_LENGTH_METERS = 1;
  public static final Angle ARM_MIN_ANGLE = PIVOT_ANGLES.get(0);
  public static final Angle ARM_MAX_ANGLE = PIVOT_ANGLES.get(PIVOT_ANGLES.size() - 1);
}
