package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static final String PHOTON_CAMERA_NAME = "photonvision"; // TODO: Update
  public static final Transform3d ROBOT_TO_CAMERA = new Transform3d();

  // for reference, this is what the photonlib example had:

  // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
  // pose, (Robot pose is considered the center of rotation at the floor level, or
  // Z = 0) and pitched 15 degrees up.

  // public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
  // new Translation3d(0.1, 0, 0.5),
  // new Rotation3d(0, Math.toDegrees(-15), 0));
}
