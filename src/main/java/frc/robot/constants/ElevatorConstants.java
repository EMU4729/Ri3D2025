package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.EncoderSupplier;

public class ElevatorConstants {
  protected ElevatorConstants() {
  }

  /** https://www.revrobotics.com/rev-11-1271/ */
  public static final int ENCODER_COUNTS_PER_REV = 8192;
  public static final double ROLLER_DIAMETER_METERS = Inches.of(1).in(Meters);
  public static final double ROLLER_CIRCUMFERENCE_METERS = Math.PI * ROLLER_DIAMETER_METERS;

  /** main elevator motor */
  public static final int MOTOR_ID = 9;
  /** encoder connected to measuring tape (genuis, btw) */
  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(
      new int[] { 6, 7 },
      ROLLER_CIRCUMFERENCE_METERS / ENCODER_COUNTS_PER_REV);

  /** distance from ground to the bottom of the ramp at lowest extension */
  public static final double GROUND_TO_RAMP_METERS = 0;

  /** p constant for elevator motor pid upper */
  public static final double PID_P = 0.1;
  /** i constant for elevator motor pid upper */
  public static final double PID_I = 0;
  /** d constant for elevator motor pid upper */
  public static final double PID_D = 0;

  public static final TrapezoidProfile.Constraints MOTION_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1);

  public static class HEIGHTS {
    public static double STOW = 0.0;
    public static double CORAL_LOAD = Units.inchesToMeters(30) - GROUND_TO_RAMP_METERS;
    public static double CORAL_L1 = Units.inchesToMeters(18) - GROUND_TO_RAMP_METERS;
    public static double CORAL_L2 = Units.inchesToMeters(31.875) - GROUND_TO_RAMP_METERS;
    public static double CORAL_L3 = Units.inchesToMeters(47.625) - GROUND_TO_RAMP_METERS;
    public static double ALGAE_PROCESSOR = 0.0;
    public static double ALGAE_L2 = Units.inchesToMeters(31.875) - GROUND_TO_RAMP_METERS;
    public static double ALGAE_L3 = Units.inchesToMeters(47.625) - GROUND_TO_RAMP_METERS;
  }

  // sim stuff
  public static final DCMotor GEARBOX = DCMotor.getFalcon500(1);
  public static final double ELEVATOR_KV = 1;
  public static final double ELEVATOR_KA = 1;
  public static final double ELEVATOR_GEARING_RATIO = 50;
  public static final double ELEVATOR_MASS = 1;
  public static final double ELEVATOR_DRUM_RADIUS = 0.5;
  public static final double MIN_HEIGHT = 0;
  public static final double MAX_HEIGHT = 2;

}
