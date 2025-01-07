package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.DriveSub;

public class TurnToAngle extends Command {
  private int finished = 0;
  private Rotation2d angle;
  private final boolean byAlliance;
  private final Rotation2d tollerance;

  public TurnToAngle(Rotation2d angle, Rotation2d tollerance) {
    this(angle, tollerance, true);
  }
  public TurnToAngle(Rotation2d angle, Rotation2d tollerance, boolean byAlliance) {
    this.angle = angle;
    this.tollerance = tollerance;
    this.byAlliance = byAlliance;
    
    addRequirements(Subsystems.drive);
  }
  
  @Override
  public void initialize() {
    if(byAlliance){angle = AutoConstants.AutoPoints.byAlliance(angle);}    
    System.out.println("Turning to Angle "+angle.getDegrees()+"deg");
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();
    DriveSub drive = Subsystems.drive;
    Rotation2d angleError = drive.calcAngleError(angle);
    drive.arcade(0, drive.calcSteer(angleError));

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