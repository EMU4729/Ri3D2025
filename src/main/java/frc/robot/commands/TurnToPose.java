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

public class TurnToPose extends Command {
  private int finished = 0;
  private double maxTurn;
  private Translation2d targetLocByTeam;
  private Translation2d targetLocOrig;
  private final boolean byAlliance;
  private final Rotation2d tollerance;

  public TurnToPose(Translation2d targetLoc, double maxTurn, Rotation2d tollerance) {
    this(targetLoc, maxTurn, tollerance, true);
  }
  public TurnToPose(Translation2d targetLoc, double maxTurn, Rotation2d tollerance, boolean byAlliance) {
    this.targetLocOrig = targetLoc; 
    this.maxTurn = maxTurn;   
    this.tollerance = tollerance;
    this.byAlliance = byAlliance;

    addRequirements(Subsystems.drive);
  }
  
  @Override
  public void initialize() {
    if(byAlliance){targetLocByTeam = AutoConstants.AutoPoints.byAlliance(targetLocOrig);}
    else {targetLocByTeam = targetLocOrig;}
    System.out.println("Turning to aim at Pose "+targetLocByTeam);
    Subsystems.drive.clearPIDError();
    super.initialize();
  }

  @Override
  public void execute() {
    DriveSub drive = Subsystems.drive;
    NavigationSub nav = Subsystems.nav;
    super.execute();

    // robot relative
    Pose2d robotPose = nav.getPose();
    Translation2d translationError = targetLocByTeam
        .minus(robotPose.getTranslation())
        .rotateBy(robotPose.getRotation().times(-1));

    Rotation2d angleError = translationError.getAngle();
    drive.arcade(0, MathUtil.clamp(drive.calcSteer(angleError), -maxTurn, maxTurn));

    if (Math.abs(angleError.getRadians()) < tollerance.getRadians()) {
      finished++;
    } else {
      finished = 0;
    }
  }

  @Override
  public boolean isFinished() {
    return finished >= 10;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping");
    Subsystems.drive.off();
    super.end(interrupted);
  }
}