package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.TemporalAmount;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;

public class DriveToPose extends Command {

  private final Pose2d targetPose;
  private final double tollerance;

  private boolean finished = false;

  public DriveToPose(Pose2d targetPose, double tollerance){
    this.targetPose = targetPose;
    this.tollerance = tollerance;
  }

  @Override
  public void initialize() {
    finished = false;
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();
    finished = Subsystems.drive.driveTo(targetPose, tollerance);
  }
  
  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.off();
    super.end(interrupted);
  }
}
