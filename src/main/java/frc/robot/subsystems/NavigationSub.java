package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.PhotonBridge;

public class NavigationSub extends SubsystemBase {
  private final ADIS16470_IMU imu = new ADIS16470_IMU();
  private final PhotonBridge photon = new PhotonBridge();
  private final Field2d field = new Field2d();

  protected final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
      DriveConstants.KINEMATICS,
      Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
      0, 0, new Pose2d());

  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);

  public NavigationSub() {
    SmartDashboard.putData(field);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getYaw() {
    return getPose().getRotation();
  }

  public Translation2d getLoc() {
    return getPose().getTranslation();
  }

  public void setCurrentPose(Pose2d newPose) {
    imu.setGyroAngle(imu.getYawAxis(), newPose.getRotation().getDegrees());
    poseEstimator.resetPosition(newPose.getRotation(),
        Subsystems.drive.getWheelPositions(), newPose);

    if (Robot.isSimulation()) {
      Subsystems.drive.drivetrainSimulator.setPose(newPose);
    }
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  @Override
  public void periodic() {
    DriveSub driveSub = Subsystems.drive;
    // TODO Auto-generated method stub
    super.periodic();

    poseEstimator.update(
        Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
        driveSub.getLeftDistance(), driveSub.getRightDistance());
    // System.out.println(imu.getAngle() +" "+ driveSub.getLeftDistance() +" "+
    // driveSub.getRightDistance() );

    if (Robot.isReal()) {
      field.setRobotPose(getPose());
    }
  }

  @Override
  public void simulationPeriodic() {
    DifferentialDrivetrainSim driveTrainSim = Subsystems.drive.drivetrainSimulator;

    photon.simulationPeriodic(driveTrainSim.getPose());
    imuSim.setGyroAngleZ(driveTrainSim.getHeading().getDegrees());
    field.setRobotPose(driveTrainSim.getPose());

    /*
     * Translation2d currentLoc = driveTrainSim.getPose().getTranslation();
     * driveTrainSim.setPose(new Pose2d(new Translation2d(
     * MathUtil.clamp(currentLoc.getX(), 0, 17.55),
     * MathUtil.clamp(currentLoc.getY(), 0, 8.05)),
     * driveTrainSim.getHeading())
     * );
     */
  }
}
