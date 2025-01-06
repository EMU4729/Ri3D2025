package frc.robot.constants;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.EncoderSupplier;

public class ElevatorConstants {
  protected ElevatorConstants() {
  }

  /** main elevator motor */
  public static final int MOTOR_ID = 2;
  /** encoder connected to measuring tape (genuis, btw) */
  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(new int[] { 0, 1 }, 999);

  /** distance from ground to the bottom of the ramp at lowest extension */
  public static final double GROUND_TO_RAMP_METERS = 0;

  /** p constant for elevator motor pid upper */
  public static final double UPPER_P = 0.1;
  /** i constant for elevator motor pid upper */
  public static final double UPPER_I = 0;
  /** d constant for elevator motor pid upper */
  public static final double UPPER_D = 0;

  public static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);

  /**
   * ground, L1, L2, L3
   * 
   * distance values from game manual pages 23-24
   */
  public static final List<Double> EXTENSION_DISTS = List.of(
      0.0,
      Units.inchesToMeters(18) - GROUND_TO_RAMP_METERS,
      Units.inchesToMeters(31.875) - GROUND_TO_RAMP_METERS,
      Units.inchesToMeters(47.625) - GROUND_TO_RAMP_METERS);

  public static final DCMotor GEARBOX = DCMotor.getFalcon500(1);
  public static final double ELEVATOR_KV = 1;
  public static final double ELEVATOR_KA = 1;
  public static final double ELEVATOR_GEARING_RATIO = 50;
  public static final double ELEVATOR_MASS = 1;
  public static final double ELEVATOR_DRUM_RADIUS = 0.5;
  public static final double MIN_HEIGHT = 0;
  public static final double MAX_HEIGHT = 2;
}
