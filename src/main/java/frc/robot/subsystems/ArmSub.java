package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.robot.constants.ArmConstants;

public class ArmSub extends SubsystemBase {
  private final TalonFX motor = new TalonFX(ArmConstants.MOTOR_ID);
  private final PositionVoltage controller = new PositionVoltage(0).withSlot(0).withFeedForward(ArmConstants.PID_FF);

  private final TalonFXSimState motorSim = motor.getSimState();
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      ArmConstants.GEARBOX,
      ArmConstants.GEARING_RATIO,
      SingleJointedArmSim.estimateMOI(ArmConstants.ARM_LENGTH_METERS, ArmConstants.ARM_MASS_KG),
      ArmConstants.ARM_LENGTH_METERS,
      ArmConstants.ARM_MIN_ANGLE.in(Radians),
      ArmConstants.ARM_MAX_ANGLE.in(Radians),
      false,
      ArmConstants.ARM_MIN_ANGLE.in(Radians));
  private final Mechanism2d mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d armTower = armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d arm = armPivot.append(
      new MechanismLigament2d(
          "Arm",
          30,
          Radians.of(armSim.getAngleRads()).in(Degrees),
          6,
          new Color8Bit(Color.kYellow)));

  public ArmSub() {
    final var motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = ArmConstants.MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

    motorConfig.Slot0.kP = ArmConstants.PID_P;
    motorConfig.Slot0.kI = ArmConstants.PID_I;
    motorConfig.Slot0.kD = ArmConstants.PID_D;

    motor.getConfigurator().apply(motorConfig);
    motor.setControl(controller);

    SmartDashboard.putData("Pivot Sim", mech2d);
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(motor.getMotorVoltage().getValueAsDouble());
    System.out.println(getCurrentAngle().in(Degrees));

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

}
