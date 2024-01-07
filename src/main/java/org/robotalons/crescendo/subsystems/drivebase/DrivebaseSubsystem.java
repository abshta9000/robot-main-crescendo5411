// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Supplier;
import java.util.Set;

import org.robotalons.crescendo.subsystems.drivebase.Constants.Devices;
import org.robotalons.lib.drivebase.DrivebaseModule;

import com.pathplanner.lib.server.PathPlannerServer;
// -------------------------------------------------------------[Robot Container]-----------------------------------------------------------//
/**
 *
 *
 * <h1>DrivebaseSubsystem</h1>
 *
 * <p>Subsystem responsible for controlling the robot's movement throughout the competition field.
 * 
 * @see SubsystemBase
 * @see {@link org.robotalons.crescendo.RobotContainer RobotContainer} 
 */
public final class DrivebaseSubsystem extends SubsystemBase {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final Set<DrivebaseModule> MODULES;
  private static final Supplier<Rotation2d> GYROSCOPE;  
  private static final SwerveDrivePoseEstimator POSE_ESTIMATOR;  
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static DrivebaseSubsystem INSTANCE = (null);

  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Drivebase Subsystem Constructor.
   */
  private DrivebaseSubsystem() {

  } static {
    INSTANCE = new DrivebaseSubsystem();
    MODULES = Set.of(
      Devices.FRONT_LEFT_MODULE,
      Devices.FRONT_RIGHT_MODULE,
      Devices.REAR_LEFT_MODULE,
      Devices.REAR_RIGHT_MODULE
    );    
    GYROSCOPE = () -> Devices.GYROSCOPE.getRotation2d();
    POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
      new SwerveDriveKinematics(
        new Translation2d( (Constants.Chassis.ROBOT_WIDTH_METERS) / (2), 
                           (Constants.Chassis.ROBOT_WIDTH_METERS) / (2)),
        new Translation2d( (Constants.Chassis.ROBOT_WIDTH_METERS) / (2),
                          -(Constants.Chassis.ROBOT_WIDTH_METERS) / (2)),
        new Translation2d(-(Constants.Chassis.ROBOT_WIDTH_METERS) / (2),
                           (Constants.Chassis.ROBOT_WIDTH_METERS) / (2)),
        new Translation2d(-(Constants.Chassis.ROBOT_WIDTH_METERS) / (2),
                          -(Constants.Chassis.ROBOT_WIDTH_METERS) / (2))), 
      GYROSCOPE.get(),
      null,   
      null
    );
    PathPlannerServer.startServer(Constants.Ports.PATHPLANNER_SERVER);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//

  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//

  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
   public static synchronized DrivebaseSubsystem getInstance() {
    if (java.util.Objects.isNull(INSTANCE)) {
      INSTANCE = new DrivebaseSubsystem();
    }
    return INSTANCE;
  }
}
