package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.NavigationSub;

public class DriveToPose extends Command {

  private Translation2d targetLoc;
  private final double targetSpeed;
  private final double tollerance;
  private final boolean byAlliance;

  private Translation2d translationError = new Translation2d(0, 0);

  public DriveToPose(Pose2d targetPose, double targetSpeed, double tollerance, boolean byAlliance) { 
    this(targetPose.getTranslation(), targetSpeed, tollerance, byAlliance);
  }
  public DriveToPose(Pose2d targetPose, double targetSpeed, double tollerance) { 
    this(targetPose.getTranslation(), targetSpeed, tollerance);
  }
  public DriveToPose(Translation2d targetLoc, double targetSpeed, double tollerance) {
    this(targetLoc, targetSpeed, tollerance, true);
  }
  public DriveToPose(Translation2d targetLoc, double targetSpeed, double tollerance, boolean byAlliance) {
    this.targetLoc = targetLoc;
    this.targetSpeed = targetSpeed;
    this.tollerance = tollerance;
    this.byAlliance = byAlliance;

    
    addRequirements(Subsystems.drive);
  }
  
  @Override
  public void initialize() {
    if(byAlliance){targetLoc = AutoConstants.AutoPoints.byAlliance(targetLoc);}
    System.out.println("Driving toward location "+targetLoc);
    translationError = new Translation2d(0, 0);
    super.initialize();
  }

  @Override
  public void execute() {
    DriveSub drive = Subsystems.drive;
    NavigationSub nav = Subsystems.nav;
    super.execute();

    // robot relative
    Pose2d robotPose = nav.getPose();
    translationError = targetLoc
        .minus(robotPose.getTranslation())
        .rotateBy(robotPose.getRotation().times(-1));
    // translationError = new Translation2d(translationError.getX(),
    // -translationError.getY());
    Rotation2d angleError = translationError.getAngle();
    double distError = translationError.getNorm();

    double steer = drive.calcSteer(angleError);
    double speed = MathUtil.clamp(drive.calcDrive(distError), -targetSpeed, targetSpeed);

    // System.out.println(speed + " " + steer);
    drive.arcade(speed, steer);
  }

  @Override
  public boolean isFinished() {
    return translationError.getX() < tollerance;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping");
    Subsystems.drive.off();
    super.end(interrupted);
  }
}
