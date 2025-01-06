package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;

public class PivotConstants {
  protected PivotConstants() {
  }

  public static final int MOTOR_ID = 3;

  // TODO: consult keith for actual angle values
  public static final List<Angle> PIVOT_ANGLES = List.of(
      Degrees.of(0), Degrees.of(35), Degrees.of(200), Degrees.of(220));

  public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

  public static final double PID_P = 0.1;
  public static final double PID_I = 0;
  public static final double PID_D = 0;
  public static final double PID_FF = 0;
}
