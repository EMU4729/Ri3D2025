package frc.robot.subsystems;

import java.time.Duration;
import java.time.Instant;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class ElevatorSub extends SubsystemBase {
  private static enum ProtectionState {
    Unset,
    Lower,
    Upper
  }

  private static final double LOWER_PROTECTION_HEIGHT = 0.05;
  private static final double UPPER_PROTECTION_HEIGHT = 0.9;

  private ProtectionState protectionState = ProtectionState.Unset;

  private final Encoder encoder = ElevatorConstants.ENCODER_ID.get();
  private final EncoderSim encoderSim = new EncoderSim(encoder);
  private final PIDController upperController = new PIDController(
      ElevatorConstants.UPPER_P,
      0,
      ElevatorConstants.UPPER_D);
  private final PIDController lowerController = new PIDController(
      ElevatorConstants.LOWER_P,
      0,
      ElevatorConstants.LOWER_D);
  private final TalonFX motor = new TalonFX(ElevatorConstants.MOTOR_ID);
  private final TalonFXSimState motorSim = motor.getSimState();
  // private final PositionVoltage liftController = new
  // PositionVoltage(0).withSlot(0);
  // private final PositionVoltage liftControllerBottom = new
  // PositionVoltage(0).withSlot(1);

  private final ElevatorSim elevatorSim;

  private final Mechanism2d mech2d;
  private final MechanismRoot2d mech2dRoot;
  private final MechanismLigament2d elevatoMech2d;

  private Instant lastTimeSim = Instant.now();

  public ElevatorSub(double pidP1, double pidD1, double pidD2, double slidingMass) {
    final var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motor.getConfigurator().apply(motorConfig);

    elevatorSim = new ElevatorSim(
        ElevatorConstants.GEARBOX,
        ElevatorConstants.ELEVATOR_GEARING_RATIO,
        ElevatorConstants.ELEVATOR_MASS,
        ElevatorConstants.ELEVATOR_DRUM_RADIUS,
        ElevatorConstants.MIN_HEIGHT,
        ElevatorConstants.MAX_HEIGHT,
        true,
        ElevatorConstants.MIN_HEIGHT,
        VecBuilder.fill(0.01));

    mech2d = new Mechanism2d(2, 3);
    mech2dRoot = mech2d.getRoot("Elevator Root", 1, 1);
    elevatoMech2d = mech2dRoot.append(
        new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  public double getPosition() {
    return encoder.getDistance();
  }

  public double getVelocity() {
    return encoder.getRate();
  }

  public void setHeight(double height) {
    elevatorSim.setState(height, 0);
    periodic();
  }

  @Override
  public void simulationPeriodic() {
    double dt = Duration.between(lastTimeSim, Instant.now()).toMillis() / 1000.0;
    lastTimeSim = Instant.now();

    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    elevatorSim.setInput(motorSim.getMotorVoltage());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(dt);

    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    encoderSim.setDistance(elevatorSim.getPositionMeters());
    encoderSim.setRate(elevatorSim.getVelocityMetersPerSecond());
    motorSim.setRawRotorPosition(elevatorSim.getPositionMeters() * ElevatorConstants.ELEVATOR_GEARING_RATIO);
    motorSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond() * ElevatorConstants.ELEVATOR_GEARING_RATIO);

    // System.out.println((double)(Math.round(liftMotor.getVelocity().getValue()*10000))/10000
    // +" "+ elevatorSim.getPositionMeters()*ratio);
    // SimBattery estimates loaded battery voltages

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
  }

  public void driveTo(double targetHeight) {
    upperController.setSetpoint(targetHeight);
    lowerController.setSetpoint(targetHeight);
  }

  @Override
  public void periodic() {
    elevatoMech2d.setLength(getPosition());

    if (!DriverStation.isEnabled()) {
      motor.set(0);
      return;
    }

    // System.out.println((double) (Math.round(elevatorSim.getPositionMeters() *
    // 10000)) / 10000 + " " +
    // (double) (Math.round(liftMotor.getPosition().getValue() * 10000)) / 10000 + "
    // " +
    // (double) (Math.round(getVelocity() * 10000)) / 10000 + " " +
    // (double) (Math.round(liftMotor.getVelocity().getValue() * 10000)) / 10000 + "
    // " +
    // (double) (Math.round(liftMotor.getMotorVoltage().getValue() * 10000)) / 10000
    // + " " +
    // targetHeight);

    double out;
    if (getPosition() < LOWER_PROTECTION_HEIGHT
        && lowerController.getSetpoint() < LOWER_PROTECTION_HEIGHT
        && protectionState != ProtectionState.Lower) {
      out = lowerController.calculate(encoder.getDistance());
      out = MathUtil.clamp(out, -1, 1);
      motor.set(out);
      protectionState = ProtectionState.Lower;
    } else if (protectionState != ProtectionState.Upper) {
      out = upperController.calculate(encoder.getDistance());
      out = MathUtil.clamp(out, -1, 1);
      motor.set(out);
      protectionState = ProtectionState.Upper;
    }

    // if (getPosition() < LOWER_PROTECTION_HEIGHT
    // && targetHeight < LOWER_PROTECTION_HEIGHT
    // && protectionState != 1) {
    // liftMotor.setControl(liftControllerBottom.withPosition(targetHeight));
    // protectionState = 1;
    /*
     * } else if(getPosition() < UPPER_PROTECTION_HEIGHT && targetHeight <
     * UPPER_PROTECTION_HEIGHT) {
     * liftMotor.setControl(liftController.withPosition(
     * UPPER_PROTECTION_HEIGHT-0.01;
     * ));
     */
    // } else if (protectionState != 3) {
    // liftMotor.setControl(liftController.withPosition(targetHeight));
    // protectionState = 3;
    // }
  }
}