package frc.robot.commands;

import java.time.Instant;

import com.fasterxml.jackson.databind.introspect.AnnotatedField;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.DriveSub;

public class DriveAtAngle extends Command {
  private Instant end;

  private final double speed;
  private final double maxTurn;
  private Rotation2d angleOrig;
  private Rotation2d angleByTeam;
  private final int runForMS;
  private final boolean byAlliance;
  
  public DriveAtAngle(double speed, double maxTurn, Rotation2d angle, int runForMS) {
    this(speed, maxTurn, angle, runForMS, true);
  }
  public DriveAtAngle(double speed, double maxTurn, Rotation2d angle, int runForMS, boolean byAlliance) {
    this.speed = speed;
    this.maxTurn = maxTurn;
    this.angleByTeam = angle;
    this.runForMS = runForMS;
    this.byAlliance = byAlliance;
  }
  
  @Override
  public void initialize() {
    if(byAlliance){angleByTeam = AutoConstants.AutoPoints.byAlliance(angleOrig);}
    else {angleByTeam = angleOrig;}
    System.out.println("Driving at Angle "+angleByTeam.getDegrees()+"deg, for "+runForMS+"ms");
    Subsystems.drive.clearPIDError();
    super.initialize();
    end = Instant.now().plusMillis(runForMS);
  }

  @Override
  public void execute() {
    super.execute();
    DriveSub drive = Subsystems.drive;
    Rotation2d angleError = drive.calcAngleError(angleByTeam);
    double tmpSpeed = Math.abs(angleError.getDegrees()) > 10 ? 0 : speed;
    double tmpSteer =  MathUtil.clamp(drive.calcSteer(angleError), -maxTurn, maxTurn);
    drive.arcade(tmpSpeed, tmpSteer);
  }

  @Override
  public boolean isFinished() {
    return Instant.now().isAfter(end);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping");

    Subsystems.drive.off();
    super.end(interrupted);
  }
}