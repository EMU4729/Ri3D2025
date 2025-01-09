package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

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
  private final TalonFX motor = new TalonFX(5);//ElevatorConstants.MOTOR_ID
  private final Encoder encoder = ElevatorConstants.ENCODER_ID.get();
  private final ProfiledPIDController controller = new ProfiledPIDController(ElevatorConstants.PID_P,
      ElevatorConstants.PID_I, ElevatorConstants.PID_D, ElevatorConstants.MOTION_CONSTRAINTS);

  VelocityVoltage driveController;

  // sim stuff
  private final TalonFXSimState motorSim = motor.getSimState();
  private final EncoderSim encoderSim = new EncoderSim(encoder);
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
  private final MechanismLigament2d elevatorMech2d = mech2dRoot
      .append(new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90));

  public ElevatorSub() {
    final var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

    
    motorConfig.Slot0.kP = 1;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;

    driveController = new VelocityVoltage(0).withSlot(0);
    motor.getConfigurator().apply(motorConfig);

    SmartDashboard.putData("Elevator Sim", mech2d);
    SmartDashboard.putData("Elevator PID", controller);

    motor.setPosition(ElevatorConstants.HEIGHTS.STOW);
  }

  public void reset(){motor.setPosition(ElevatorConstants.HEIGHTS.STOW);}

  public double getHeight() {
    return encoder.getDistance();
  }

  public double getVelocity() {
    return encoder.getRate();
  }

  public void setSimHeight(double height) {
    elevatorSim.setState(height, 0);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    elevatorSim.setInput(motorSim.getMotorVoltage());

    // Next, we update it. The standard loop time is 20ms.
    elevatorSim.update(0.02);

    // Next, we update it. The standard loop time is 20ms.
    // System.out.println(elevatorSim.getPositionMeters());
    // System.out.println(motorSim.getMotorVoltage());

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
    //elevatorMech2d.setLength(getHeight());

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

    // System.out.println(controller.getP());
    // System.out.println(controller.getI());
    // System.out.println(controller.getD());
    // System.out.println(controller.getConstraints().maxAcceleration);
    // System.out.println(controller.getConstraints().maxVelocity);
    // System.out.println();

    var out = controller.calculate(getHeight());
    out = MathUtil.clamp(out, -1, 1);
    motor.set(out);
  }
}