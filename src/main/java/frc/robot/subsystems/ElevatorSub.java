package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
  private final Encoder encoder = ElevatorConstants.ENCODER_ID.get();
  private final EncoderSim encoderSim = new EncoderSim(encoder);

  private final ProfiledPIDController controller = new ProfiledPIDController(ElevatorConstants.UPPER_P,
      ElevatorConstants.UPPER_I, ElevatorConstants.UPPER_D, ElevatorConstants.MOTION_CONSTRAINTS);

  private final TalonFX motor = new TalonFX(ElevatorConstants.MOTOR_ID);
  private final TalonFXSimState motorSim = motor.getSimState();

  private final ElevatorSim elevatorSim = new ElevatorSim(
      ElevatorConstants.GEARBOX,
      ElevatorConstants.ELEVATOR_GEARING_RATIO,
      ElevatorConstants.ELEVATOR_MASS,
      ElevatorConstants.ELEVATOR_DRUM_RADIUS,
      ElevatorConstants.MIN_HEIGHT,
      ElevatorConstants.MAX_HEIGHT,
      true,
      ElevatorConstants.MIN_HEIGHT,
      0.01,
      0.01);

  private final Mechanism2d mech2d = new Mechanism2d(20, 30);
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 1);
  private final MechanismLigament2d elevatoMech2d = mech2dRoot
      .append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

  public ElevatorSub() {
    final var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motor.getConfigurator().apply(motorConfig);

    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  public double getHeight() {
    return encoder.getDistance();
  }

  public double getVelocity() {
    return encoder.getRate();
  }

  public void setSimHeight(double height) {
    elevatorSim.setState(height, 0);
    periodic();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    elevatorSim.setInput(motorSim.getMotorVoltage());
    elevatorSim.update(0.02);
    

    // Next, we update it. The standard loop time is 20ms.
    System.out.println(elevatorSim.getPositionMeters());
    System.out.println(motorSim.getMotorVoltage());

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

  public void setTargetHeight(double targetHeight) {
    controller.setGoal(targetHeight);
  }

  public double getTargetHeight() {
    return controller.getGoal().position;
  }

  @Override
  public void periodic() {
    elevatoMech2d.setLength(getHeight());

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

    var out = controller.calculate(getHeight());
    out = MathUtil.clamp(out, -1, 1);
    motor.set(out);
  }
}