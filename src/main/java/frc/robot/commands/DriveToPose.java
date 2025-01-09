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

  private Translation2d targetLocByTeam;
  private Translation2d targetLocOrig;
  private final double targetSpeed;
  private final double maxTurn;
  private final double tollerance;
  private final boolean byAlliance;
  private int finished = 0;

  private Translation2d translationError = new Translation2d(0, 0);

  public DriveToPose(Pose2d targetPose, double targetSpeed, double  maxTurn, double tollerance, boolean byAlliance) { 
    this(targetPose.getTranslation(), targetSpeed, maxTurn, tollerance, byAlliance);
  }
  public DriveToPose(Pose2d targetPose, double targetSpeed, double  maxTurn, double tollerance) { 
    this(targetPose.getTranslation(), targetSpeed, maxTurn, tollerance);
  }
  public DriveToPose(Translation2d targetLoc, double targetSpeed, double  maxTurn, double tollerance) {
    this(targetLoc, targetSpeed, maxTurn, tollerance, true);
  }
  public DriveToPose(Translation2d targetLoc, double targetSpeed, double  maxTurn, double tollerance, boolean byAlliance) {
    this.targetLocOrig = targetLoc;
    this.targetSpeed = targetSpeed;
    this.maxTurn = maxTurn;
    this.tollerance = tollerance;
    this.byAlliance = byAlliance;

    
    addRequirements(Subsystems.drive);
  }
  
  @Override
  public void initialize() {
    if(byAlliance){targetLocByTeam = AutoConstants.AutoPoints.byAlliance(targetLocOrig);}
    else {targetLocByTeam = targetLocOrig;}
    System.out.println("Driving toward location "+targetLocByTeam);
    Subsystems.drive.clearPIDError();
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
    translationError = targetLocByTeam
        .minus(robotPose.getTranslation())
        .rotateBy(robotPose.getRotation().times(-1));
    // translationError = new Translation2d(translationError.getX(),
    // -translationError.getY());
    Rotation2d angleError = new Translation2d(Math.max(translationError.getX(), 1), 
                                              Math.min(translationError.getY(), 1)).getAngle();
    double distError = translationError.getNorm();

    double steer = MathUtil.clamp(drive.calcSteer(angleError), -maxTurn, maxTurn);
    double speed = MathUtil.clamp(drive.calcDrive(distError), -targetSpeed, targetSpeed);

    // System.out.println(speed + " " + steer);
    drive.arcade(speed, steer);

    if(translationError.getX() < tollerance){
      finished++;
    } else {
      finished = 0;
    }
  }

  @Override
  public boolean isFinished() {
    return finished > 20;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping");
    Subsystems.drive.off();
    super.end(interrupted);
  }
}
