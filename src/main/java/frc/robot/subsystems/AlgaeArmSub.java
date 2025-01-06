package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeArmConstants;

public class AlgaeArmSub extends SubsystemBase {
  private final TalonFX motor = new TalonFX(AlgaeArmConstants.MOTOR_ID);
  private final PositionDutyCycle controller = new PositionDutyCycle(0).withSlot(0)
      .withFeedForward(AlgaeArmConstants.PID_FF);

  // sim stuff
  private final TalonFXSimState motorSim = motor.getSimState();
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      AlgaeArmConstants.GEARBOX,
      AlgaeArmConstants.GEARING_RATIO,
      SingleJointedArmSim.estimateMOI(AlgaeArmConstants.ARM_LENGTH_METERS, AlgaeArmConstants.ARM_MASS_KG),
      AlgaeArmConstants.ARM_LENGTH_METERS,
      AlgaeArmConstants.ARM_MIN_ANGLE.in(Radians),
      AlgaeArmConstants.ARM_MAX_ANGLE.in(Radians),
      false,
      AlgaeArmConstants.ARM_MIN_ANGLE.in(Radians));
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mech2d.getRoot("AlgaeArmPivot", 30, 30);
  private final MechanismLigament2d armTower = armPivot.append(new MechanismLigament2d("AlgaeArmTower", 30, -90));
  private final MechanismLigament2d arm = armPivot.append(
      new MechanismLigament2d(
          "AlgaeArm",
          30,
          Radians.of(armSim.getAngleRads()).in(Degrees),
          6,
          new Color8Bit(Color.kYellow)));

  public AlgaeArmSub() {
    final var motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = AlgaeArmConstants.MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

    motorConfig.Slot0.kP = AlgaeArmConstants.PID_P;
    motorConfig.Slot0.kI = AlgaeArmConstants.PID_I;
    motorConfig.Slot0.kD = AlgaeArmConstants.PID_D;

    motor.getConfigurator().apply(motorConfig);
    motor.setControl(controller);

    SmartDashboard.putData("Algae Arm Sim", mech2d);
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(motor.getMotorVoltage().getValueAsDouble());
    armSim.update(0.02);

    // note: we assume mechanism rotations and rotor rotations are equivalent.
    // make sure to change this in the future, once we have an actual gearing ratio.
    motorSim.setRawRotorPosition(Radians.of(armSim.getAngleRads()).in(Rotations));

    arm.setAngle(Radians.of(armSim.getAngleRads()).in(Degrees));
  }

  public Angle getTargetAngle() {
    final var controlRequest = (PositionVoltage) motor.getAppliedControl();
    return controlRequest.getPositionMeasure();
  }

  public Angle getCurrentAngle() {
    return motor.getPosition().getValue();
  }

  public void setTargetAngle(Angle angle) {
    motor.setControl(controller.withPosition(angle));
  }

  public void setTargetAngle(Angle angle, double maxSpeed) {
    motor.setControl(controller.withVelocity(maxSpeed).withPosition(angle));
  }

}
