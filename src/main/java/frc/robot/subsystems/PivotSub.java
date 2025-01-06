package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;

public class PivotSub extends SubsystemBase {
  private final TalonFX motor = new TalonFX(PivotConstants.MOTOR_ID);
  private final PositionVoltage controller = new PositionVoltage(0).withSlot(0).withFeedForward(PivotConstants.PID_FF);

  public PivotSub() {
    final var motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = PivotConstants.MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

    motorConfig.Slot0.kP = PivotConstants.PID_P;
    motorConfig.Slot0.kI = PivotConstants.PID_I;
    motorConfig.Slot0.kD = PivotConstants.PID_D;

    motor.getConfigurator().apply(motorConfig);
    motor.setControl(controller);
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
