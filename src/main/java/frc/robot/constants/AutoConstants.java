package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.utils.PIDControllerSupplier;

public class AutoConstants {
  // TODO: tune these
  /** Auto translation PID constants */
  public static final PIDControllerSupplier TRANSLATION_PID = new PIDControllerSupplier(0.5, 0, 0);
  /** Auto rotation PID constants */
  public static final PIDControllerSupplier ROT_PID = new PIDControllerSupplier(0.5, 0, 0);
  /** Tolerance for translation PID in meters */
  public static final double TRANSLATION_TOLERANCE = 0.1;
  /** Tolerance for rotation PID in radians */
  public static final double ROT_TOLERANCE = 0.1;

  /** all names rel to drive team
   * all numbers for blue alliance
   * 
   */
  public static class AutoPoints {
    public static final Translation2d FIELD_SIZE = 
            new Translation2d(17.54823, 8.0518);
    public static final Translation2d FIELD_CENTRE = 
            new Translation2d(FIELD_SIZE.getX()/2, FIELD_SIZE.getY()/2);
    public static final Translation2d ROBOT_SIZE = 
            new Translation2d(0.820, 0.635);
    public static final Translation2d ROBOT_SIZE_BUMPERS = 
            ROBOT_SIZE.plus(new Translation2d(0.15, 0.15));
    public static final Translation2d REEF_LOC =
            new Translation2d(4.489, FIELD_CENTRE.getY());

    /** centred on the line below the tag */
    public static final Pose2d START_LEFT = 
            new Pose2d(7.557, 6.138, Rotation2d.k180deg);
    /** centred on the line against the truss */
    public static final Pose2d START_CENTRE =
            new Pose2d(FIELD_CENTRE.getX() - ROBOT_SIZE_BUMPERS.getX()/2 - 0.1524, FIELD_CENTRE.getY(),
            Rotation2d.k180deg);
    /** centred on the line below the tag */
    public static final Pose2d START_RIGHT = 
            new Pose2d( 7.557, 1.915, Rotation2d.k180deg);

    private static final Translation2d REEF_BOT_OFFSET = new Translation2d(0.83185 + ROBOT_SIZE_BUMPERS.getX()/2, 0);
    private static final int[] REEF_ANGLES = {300, 240, 0, 180, 60, 120};
    public static final Pose2d REEF_NEAR_LEFT =
            new Pose2d(REEF_LOC.minus(REEF_BOT_OFFSET.rotateBy(Rotation2d.fromDegrees(REEF_ANGLES[0]))), Rotation2d.fromDegrees(REEF_ANGLES[0]));
    public static final Pose2d REEF_FAR_LEFT =
            new Pose2d(REEF_LOC.minus(REEF_BOT_OFFSET.rotateBy(Rotation2d.fromDegrees(REEF_ANGLES[1]))), Rotation2d.fromDegrees(REEF_ANGLES[1]));
    public static final Pose2d REEF_NEAR_CENTRE = 
            new Pose2d(REEF_LOC.minus(REEF_BOT_OFFSET.rotateBy(Rotation2d.fromDegrees(REEF_ANGLES[2]))), Rotation2d.fromDegrees(REEF_ANGLES[2]));
    public static final Pose2d REEF_FAR_CENTRE =
            new Pose2d(REEF_LOC.minus(REEF_BOT_OFFSET.rotateBy(Rotation2d.fromDegrees(REEF_ANGLES[3]))), Rotation2d.fromDegrees(REEF_ANGLES[3]));
    public static final Pose2d REEF_NEAR_RIGHT =
            new Pose2d(REEF_LOC.minus(REEF_BOT_OFFSET.rotateBy(Rotation2d.fromDegrees(REEF_ANGLES[4]))), Rotation2d.fromDegrees(REEF_ANGLES[4]));
    public static final Pose2d REEF_FAR_RIGHT =
            new Pose2d(REEF_LOC.minus(REEF_BOT_OFFSET.rotateBy(Rotation2d.fromDegrees(REEF_ANGLES[5]))), Rotation2d.fromDegrees(REEF_ANGLES[5]));
            
    public static final Translation2d PROCESSOR = new Translation2d(5.987542, ROBOT_SIZE_BUMPERS.getY()/2);
    
    public static final Translation2d ALGAE_LEFT = new Translation2d(1.2192, FIELD_CENTRE.getY() + 1.8288);
    public static final Translation2d ALGAE_CENTRE = new Translation2d(1.2192, FIELD_CENTRE.getY());
    public static final Translation2d ALGAE_RIGHT = new Translation2d(1.2192, FIELD_CENTRE.getY() - 1.8288);
    

    private static final Translation2d FEEDER_BOT_OFFSET = new Translation2d(0.00381 + ROBOT_SIZE_BUMPERS.getX()/2, 0);
    private static final int[] FEEDER_ANGLES = {240, 120};
    public static final Pose2d FEEDER_RIGHT =
            new Pose2d(new Translation2d(0.851154, 0.65532).minus(FEEDER_BOT_OFFSET.rotateBy(Rotation2d.fromDegrees(FEEDER_ANGLES[0]))), Rotation2d.fromDegrees(FEEDER_ANGLES[0]));
    public static final Pose2d FEEDER_LEFT =
            new Pose2d(new Translation2d(0.851154, 7.39648).minus(FEEDER_BOT_OFFSET.rotateBy(Rotation2d.fromDegrees(FEEDER_ANGLES[1]))), Rotation2d.fromDegrees(FEEDER_ANGLES[1]));
    
    //----------------------------------------------------------------

    public static Pose2d byAlliance(Pose2d pose){
      if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){return pose;}
      Translation2d newLoc = pose.getTranslation().rotateAround(FIELD_CENTRE, Rotation2d.k180deg);
      Rotation2d newRot = pose.getRotation().rotateBy(Rotation2d.k180deg);
      
      return new Pose2d(newLoc, newRot);
    }
    public static Translation2d byAlliance(Translation2d loc){
      if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){return loc;}
      return loc.rotateAround(FIELD_CENTRE, Rotation2d.k180deg);
    }
    public static Rotation2d byAlliance(Rotation2d rot){
      if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){return rot;}
      return rot.rotateBy(Rotation2d.k180deg);
    }
  } 
}
