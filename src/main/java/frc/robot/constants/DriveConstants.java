package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.EncoderSupplier;
import frc.robot.utils.RangeMath.AxesFit;
import frc.robot.utils.RangeMath.DriveBaseFit;
import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonMotorSupplier;
import frc.robot.utils.motorsupplier.VictorMotorSupplier;

public class DriveConstants {
  protected DriveConstants() {
  }

  /**
   * Information for left master drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public static final MotorSupplier<WPI_VictorSPX> MOTOR_ID_LM = new VictorMotorSupplier(1)
      .withSafety().withInvert();

  /** Drive left encoder builder */
  public static final EncoderSupplier ENCODER_ID_L = new EncoderSupplier(new int[] { 0, 1 }, 60.078 / 256. / 1000);

  /**
   * Information for right master drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public static final MotorSupplier<WPI_VictorSPX> MOTOR_ID_RM = new VictorMotorSupplier(3)
      .withSafety();

  /** Drive left encoder builder */
  public static final EncoderSupplier ENCODER_ID_R = new EncoderSupplier(new int[] { 2, 3 }, 59.883 / 256. / 1000)
      .withInvert();

  /**
   * Information for left slave drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public static final MotorSupplier<WPI_VictorSPX> MOTOR_ID_LS = new VictorMotorSupplier(2)
      .withInvert();
  /**
   * Information for right slave drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public static final MotorSupplier<WPI_VictorSPX> MOTOR_ID_RS = new VictorMotorSupplier(4);

  /* Whether to clamp drive/turn before setting the motors */
  public static final boolean USE_CLAMPING = true;

  /** KS value from SysId */
  public static final double KS_VOLTS = 0.88881;
  /** KV value from SysId */
  public static final double KV_VOLT_SECONDS_PER_METER = 3.0288;
  /** KA value from SysId */
  public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 1.036;
  /** Horizontal distance between the drive wheels, in meters */
  public static final double TRACK_WIDTH_METERS = 0.55;
  /** Drive kinematics */
  public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
      TRACK_WIDTH_METERS);
  /** Auto max velocity */
  public static final double AUTO_MAX_SPEED = 2;
  /** Auto max acceleration */
  public static final double AUTO_MAX_ACCELERATION = 1;
  /** Auto ramsete b variable */
  public static final double RAMSETE_B = 2;
  /** Auto ramsete zeta variable */
  public static final double RAMSETE_ZETA = 0.7;

  // Drive Settings
  /** max speed of robot m/s */
  public static double MAX_SPEED = 3.850;
  /** min throttle for movement */
  public static final double MIN_THROT = 0.3;
  /** min throttle for turning */
  public static final double MIN_TURN = 0.3;

  /** Distance between front and back wheel on robot in meters */
  public static final double TRACK_WIDTH = Units.inchesToMeters(20.7);
  /** Distance between centers of left and right wheels on robot in meters */
  public static final double WHEEL_BASE = Units.inchesToMeters(20.7);
  /** Wheel diameter in meters */
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);

  /**
   * settings for robot drive in default teleop
   * 
   * modRFromX ?
   * set to zero for now
   */
  /*
   * public static final DriveBaseFit PILOT_SETTINGS = DriveBaseFit.InitTankBot(
   * MIN_THROT, 1, 2, 0.1, false,
   * MIN_TURN, 1, 3, 0.1, false,
   * 0.5, 0, 1);
   */
  public static final DriveBaseFit PILOT_SETTINGS = new DriveBaseFit(
      new AxesFit().withOutputMinMax(MIN_THROT, 0.8).withDeadBand(0.1).withPow(2).withBooster(1).withLimiter(0.5),
      new AxesFit().withOutputMinMax(MIN_TURN, 0.7).withDeadBand(0.1).withPow(3),
      0.5).invertX().invertYaw();

  /**
   * settings for robot drive in demo mode
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  /*
   * public static final DriveBaseFit DEMO_SETTINGS = DriveBaseFit.InitTankBot(
   * MIN_THROT, 0.5, 3, 0.1, false,
   * MIN_TURN, 0.6, 4, 0.1, false,
   * 0.5, 0, 1);
   */
  public static final DriveBaseFit DEMO_SETTINGS = new DriveBaseFit(
      new AxesFit().withOutputMinMax(MIN_THROT, 0.5).withDeadBand(0.1).withPow(3),
      new AxesFit().withOutputMinMax(MIN_TURN, 0.6).withDeadBand(0.1).withPow(4),
      0.5).invertX().invertYaw();

  /**
   * settings for robot drive in PID drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  /*
   * public static final DriveBaseFit PID1_SETTINGS = DriveBaseFit.InitTankBot(
   * 0, MAX_SPEED, 3, 0, false,
   * 0, 1, 3, 0, false,
   * 0.5, 0, 1);
   */

  /**
   * settings for robot drive in PID drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  /*
   * public static final DriveBaseFit PID2_SETTINGS = DriveBaseFit.InitTankBot(
   * MIN_THROT, 1, 1, 0, false,
   * MIN_TURN, 1, 1, 0, false,
   * 0.5, 0, 1);
   */
}
