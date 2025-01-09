package frc.robot.commands;

import javax.crypto.Mac;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.DriveSub;

public class TurnToAngle extends Command {
  private int finished = 0;
  private final double maxTurn;
  private Rotation2d angleOrig;
  private Rotation2d angleTeamRel;
  private final boolean byAlliance;
  private final Rotation2d tollerance;

  public TurnToAngle(Rotation2d angle, double maxTurn, Rotation2d tollerance) {
    this(angle, maxTurn, tollerance, true);
  }
  public TurnToAngle(Rotation2d angle, double maxTurn, Rotation2d tollerance, boolean byAlliance) {
    this.angleOrig = angle;
    this.maxTurn = maxTurn;
    this.tollerance = tollerance;
    this.byAlliance = byAlliance;
    
    addRequirements(Subsystems.drive);
  }
  
  @Override
  public void initialize() {
    if(byAlliance){angleTeamRel = AutoConstants.AutoPoints.byAlliance(angleOrig);}
    else {angleTeamRel = angleOrig;}
    System.out.println("Turning to Angle "+angleTeamRel.getDegrees()+"deg");
    Subsystems.drive.clearPIDError();
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();
    DriveSub drive = Subsystems.drive;
    Rotation2d angleError = drive.calcAngleError(angleTeamRel);
    drive.arcade(0, MathUtil.clamp(drive.calcSteer(angleError), -maxTurn, maxTurn));

    if (Math.abs(angleError.getRadians()) < tollerance.getRadians()) {
      finished++;
    } else {
      finished = 0;
    }
  }

  @Override
  public boolean isFinished() {
    return finished >= 20;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping");
    Subsystems.drive.off();
    super.end(interrupted);
  }
}