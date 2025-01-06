package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SimConstants;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DriveSub extends SubsystemBase {
  private final WPI_TalonSRX leftMaster = DriveConstants.MOTOR_ID_LM.get();
  private final WPI_TalonSRX leftSlave = DriveConstants.MOTOR_ID_LS.get();

  private final WPI_TalonSRX rightMaster = DriveConstants.MOTOR_ID_RM.get();
  private final WPI_TalonSRX rightSlave = DriveConstants.MOTOR_ID_RS.get();

  private final Encoder leftEncoder = DriveConstants.ENCODER_ID_L.get();
  private final Encoder rightEncoder = DriveConstants.ENCODER_ID_R.get();
  public final DifferentialDrive drive; // pub for shuffleboard

  public final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  public final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
  public final Field2d field = new Field2d();

  // Simulation Variables
  /** @wip add corrected values */
  private final LinearSystem<N2, N2, N2> drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(
      SimConstants.KV_LINEAR,
      SimConstants.KA_LINEAR,
      SimConstants.KV_ANGULAR,
      SimConstants.KA_ANGULAR);

  private final double randomBiasSim = (Math.random() - 0.5) / 6;

  private final PIDController steerPID = new PIDController(0.025, 0.0001, 0.004);
  private final PIDController drivePID = new PIDController(0.5, 0.01, 0.01);

  private double driveThrottle;
  private double turnThrottle;

  public final DifferentialDrivetrainSim drivetrainSimulator = new DifferentialDrivetrainSim(
      drivetrainSystem, DCMotor.getCIM(2), 10.71, DriveConstants.WHEEL_BASE,
      DriveConstants.WHEEL_DIAMETER_METERS / 2, null);

  public DriveSub() {
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    drive = new DifferentialDrive(leftMaster, rightMaster);

    SmartDashboard.putData(field);

    addChild("Differential Drive", drive);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public DifferentialDriveWheelPositions getWheelPositions() {
    return new DifferentialDriveWheelPositions(leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  /**
   * Tank drive.
   * 
   * @param leftSpeed  The left speed.
   * @param rightSpeed The right speed.
   */
  public void tank(double leftSpeed, double rightSpeed) {
    leftSpeed = MathUtil.clamp(simNoise(leftSpeed), -1, 1);
    rightSpeed = MathUtil.clamp(simNoise(rightSpeed), -1, 1);
    drive.tankDrive(leftSpeed, rightSpeed, false);
  }

  /**
   * Tank drives the robot using the specified voltages.
   * <strong>Highly unsafe.</strong> Values are uncapped, so use with caution.
   * 
   * @param leftVoltage  The left output
   * @param rightVoltage The right output
   */
  public void tankVoltage(double leftVoltage, double rightVoltage) {
    leftMaster.setVoltage(leftVoltage);
    rightMaster.setVoltage(rightVoltage);
    drive.feed();
  }

  /**
   * Arcade drive.
   * 
   * @param throttle The speed
   * @param steering The steering
   */
  public void arcade(double throttle, double steering) {
    if (throttle != 0 || steering != 0) {
      steering = simNoise(steering);
    }
    if (DriveConstants.USE_CLAMPING) {
      driveThrottle = MathUtil.clamp(driveThrottle, -0.4, 0.4);
      turnThrottle = MathUtil.clamp(turnThrottle, -0.8, 0.8);
    }
    this.driveThrottle = throttle;
    this.turnThrottle = steering;

    drive.arcadeDrive(throttle, -steering, true); // squared input fix later
  }

  /** Stops all motors. */
  public void off() {
    tank(0, 0);
    driveThrottle = 0;
    turnThrottle = 0;
  }

  @Override
  public void simulationPeriodic() {
    // set sim motor volts to cur motor throt * bat volts

    // the order is reversed because otherwise, simulation direction is the opposite
    // of real life
    // this is probably a result of a deeper issue with the code that i don't want
    // to fix right now
    drivetrainSimulator.setInputs(
        rightMaster.get() * RobotController.getInputVoltage(),
        leftMaster.get() * RobotController.getInputVoltage());
    drivetrainSimulator.update(0.02);

    leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
    leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
    rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());

  }

  /* SysId routine for drive */
  /*
   * public final SysIdRoutine sysIdDrive = new SysIdRoutine(
   * new SysIdRoutine.Config(
   * Units.Volts.per(Units.Second).of(0.2),
   * Units.Volt.of(0.4),
   * Units.Second.of(6)),
   * new SysIdRoutine.Mechanism(new Consumer<Voltage>() {
   * 
   * @Override
   * public void accept(Voltage value) {
   * arcade(value.in(Units.Volt), 0);
   * }
   * }, new Consumer<SysIdRoutineLog>() {
   * 
   * @Override
   * public void accept(SysIdRoutineLog log) {
   * log.motor("drive")
   * .voltage(Units.Volt.of(driveThrottle))
   * .linearPosition(Units.Meter.of(leftEncoder.getDistance()))
   * .linearVelocity(Units.MetersPerSecond.of(leftEncoder.getRate()))
   * .linearAcceleration(Units.MetersPerSecondPerSecond.of(imu.getAccelX()));
   * }
   * }, this));
   */

  /* SysId routine for turn */
  /*
   * public final SysIdRoutine sysIdTurn = new SysIdRoutine(
   * new SysIdRoutine.Config(
   * Units.Volts.per(Units.Second).of(0.1),
   * Units.Volt.of(0.4),
   * Units.Second.of(6)),
   * new SysIdRoutine.Mechanism(new Consumer<Voltage>() {
   * 
   * @Override
   * public void accept(Voltage value) {
   * arcade(0, value.in(Units.Volt));
   * }
   * }, new Consumer<SysIdRoutineLog>() {
   * 
   * @Override
   * public void accept(SysIdRoutineLog log) {
   * log.motor("turn")
   * .voltage(Units.Volt.of(turnThrottle))
   * .angularPosition(Units.Degrees.of(imu.getAngle(imu.getYawAxis())))
   * .angularVelocity(Units.DegreesPerSecond.of(imu.getRate(imu.getYawAxis())))
   * .angularAcceleration(Units.DegreesPerSecond.per(Units.Second).of(imu.
   * getYFilteredAccelAngle()));
   * }
   * }, this));
   */

  private double simNoise(double in) {
    if (Robot.isReal()) {
      return in;
    }
    return in + ((Math.random() - 0.5) / 5) + randomBiasSim;
  }

  public double calcDrive(double distError) {
    return -drivePID.calculate(distError);
  }

  public double calcSteer(Rotation2d angleError) {
    return -steerPID.calculate(angleError.getDegrees());
  }

  public Rotation2d calcAngleError(Rotation2d targetAngle) {
    return targetAngle.minus(Subsystems.nav.getYaw());
  }
}