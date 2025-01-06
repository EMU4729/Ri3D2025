package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.TemporalAmount;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.NavigationSub;

public class TurnToPose extends Command {
  private int finished = 0;
  private final Translation2d targetLoc;
  private final Rotation2d tollerance;

  public TurnToPose(Translation2d targetLoc, Rotation2d tollerance){
    this.targetLoc = targetLoc;
    this.tollerance = tollerance;
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    DriveSub drive = Subsystems.drive;
    NavigationSub nav = Subsystems.nav;
    super.execute();

    // robot relative
    Pose2d robotPose = nav.getPose();
    Translation2d translationError = targetLoc
                                              .minus(robotPose.getTranslation())
                                              .rotateBy(robotPose.getRotation().times(-1));

    Rotation2d angleError = translationError.getAngle();
    drive.arcade(0, drive.calcSteer(angleError));

    if(Math.abs(angleError.getRadians()) < tollerance.getRadians()){
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
    Subsystems.drive.off();
    super.end(interrupted);
  }
}