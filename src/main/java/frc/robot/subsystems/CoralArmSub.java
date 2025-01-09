package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AlgaeArmConstants;
import frc.robot.constants.CoralArmConstants;

public class CoralArmSub extends SubsystemBase {
  //private final WPI_TalonSRX motor = CoralArmConstants.MOTOR_ID.get();
  private final TalonFX motor = new TalonFX(CoralArmConstants.MOTOR_ID.port);
  //private final Encoder encoder = CoralArmConstants.ENCODER_ID.get();


  /*private final PIDController controller = new PIDController(
      CoralArmConstants.PID_P,
      CoralArmConstants.PID_I,
      CoralArmConstants.PID_D);*/
  private final PositionVoltage controller = new PositionVoltage(0).withSlot(0)
      .withFeedForward(CoralArmConstants.PID_FF);

  public CoralArmSub() {
    final var motorConfig = new TalonFXConfiguration();

    motorConfig.MotorOutput.Inverted = CoralArmConstants.MOTOR_INVERTED;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

    motorConfig.Slot0.kP = CoralArmConstants.PID_P;
    motorConfig.Slot0.kI = CoralArmConstants.PID_I;
    motorConfig.Slot0.kD = CoralArmConstants.PID_D;

    motor.getConfigurator().apply(motorConfig);
    motor.setControl(controller);
    motor.setPosition(CoralArmConstants.ANGLES.STOW);
    setTargetAngle(CoralArmConstants.ANGLES.STOW);
    //controller.setSetpoint(0);
  }
  public void reset(){
    motor.setPosition(CoralArmConstants.ANGLES.STOW);
  }

  public double getCurrentAngle() {
    //return encoder.getDistance();
    return motor.getPosition().getValue().in(Degrees);
  }

  /*
  public void setTargetAngle(Angle targetAngle) {
    setTargetAngle(targetAngle);
  }*/

  public void setTargetAngle(Angle targetAngleDegrees) {
    //controller.setSetpoint(targetAngleDegrees);
    motor.setControl(controller.withPosition(targetAngleDegrees.times(50)));
  }

 /* public double getTargetAngle() {
    //return controller.getSetpoint();
    return motor.setpo
    
  }*/
/*
  @Override
  public void periodic() {
    var out = controller.calculate(getCurrentAngle());

    out = MathUtil.clamp(out, -1, 1);
    motor.set(out);
  }*/
}
