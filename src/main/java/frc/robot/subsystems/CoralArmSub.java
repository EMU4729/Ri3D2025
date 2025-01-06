package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CoralArmConstants;

public class CoralArmSub extends SubsystemBase {
  private final WPI_TalonSRX motor = CoralArmConstants.MOTOR_ID.get();
  private final Encoder encoder = CoralArmConstants.ENCODER_ID.get();

  private final PIDController controller = new PIDController(
      CoralArmConstants.PID_P,
      CoralArmConstants.PID_I,
      CoralArmConstants.PID_D);

  public CoralArmSub() {
    controller.setSetpoint(0);
  }

  public double getCurrentAngle() {
    return encoder.getDistance();
  }

  public void setTargetAngle(Angle targetAngle) {
    setTargetAngle(targetAngle.in(Degrees));
  }

  public void setTargetAngle(double targetAngleDegrees) {
    controller.setSetpoint(targetAngleDegrees);
  }

  public double getTargetAngle() {
    return controller.getSetpoint();
  }

  @Override
  public void periodic() {
    var out = controller.calculate(getCurrentAngle());
    out = MathUtil.clamp(out, -1, 1);
    motor.set(out);
  }
}
