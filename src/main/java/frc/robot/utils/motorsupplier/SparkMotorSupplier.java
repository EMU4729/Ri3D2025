package frc.robot.utils.motorsupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SparkMotorSupplier extends MotorSupplier<CANSparkMax> {

  public SparkMotorSupplier(int port) {
    super(port);
  }

  @Override
  public MotorSupplier<CANSparkMax> withBrake() {
    throw new UnsupportedOperationException();
  }

  @Override
  public MotorSupplier<CANSparkMax> withSafety() {
    throw new UnsupportedOperationException();
  }

  public CANSparkMax get() {
    if (port < 0) {
      System.out.println("MotorInfo : motor port num < 0, check port is defined : " + port);
      return new CANSparkMax(port, MotorType.kBrushed);
    }
    CANSparkMax spark = new CANSparkMax(port, MotorType.kBrushed);

    spark.setInverted(invert);

    if (voltageComp) {
      spark.enableVoltageCompensation(12);
    } else {
      spark.disableVoltageCompensation();
    }

    return spark;
  }
}
