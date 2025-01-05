package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import java.time.temporal.TemporalAmount;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.NavigationSub;

public class DriveAtAngle extends Command {
  private Instant end; 

  private final double speed;
  private final Rotation2d angle;
  private final int runForMS;

  public DriveAtAngle(double speed, Rotation2d angle, int runForMS){
    this.speed = speed;
    this.angle = angle;
    this.runForMS = runForMS;
  }

  @Override
  public void initialize() {
    super.initialize();
    end = Instant.now().plusMillis(runForMS);
  }

  @Override
  public void execute() {
    super.execute();
    DriveSub drive = Subsystems.drive;
    Rotation2d angleError = drive.calcAngleError(angle);
    drive.arcade(angleError.getDegrees() > 10 ? 0:speed, drive.calcSteer(angleError));
  }
  
  @Override
  public boolean isFinished() {
    return Instant.now().isAfter(end);
  }

  @Override
  public void end(boolean interrupted) {
    Subsystems.drive.off();
    super.end(interrupted);
  }
}