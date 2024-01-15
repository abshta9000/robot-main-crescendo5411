// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo.subsystems.drivebase;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import org.littletonrobotics.junction.Logger;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseConstants.Devices;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseConstants.Measurements;
import org.robotalons.crescendo.subsystems.drivebase.DrivebaseConstants.Objects;
import org.robotalons.lib.kinematics.Gyroscope;
import org.robotalons.lib.motion.Module;
import org.robotalons.lib.odometry.LocalADStarAK;

import java.io.Closeable;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.IntStream;
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
public final class DrivebaseSubsystem extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final SwerveDrivePoseEstimator POSE_ESTIMATOR;
  private static final SwerveDriveKinematics KINEMATICS;  
  private static final List<Module> MODULES;  
  private static final Gyroscope GYROSCOPE;  
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static DrivebaseSubsystem Instance;
  private static Rotation2d Odometry_Rotation;
  private static Pose2d Odometry_Pose;  
  private static Boolean Module_Locking;      
  private static Boolean Path_Flipped;  
  private static Double Current_Time;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Drivebase Subsystem Constructor.
   */
  public DrivebaseSubsystem() {} static {
    Instance = new DrivebaseSubsystem();
    Odometry_Pose = new Pose2d();
    GYROSCOPE = Devices.GYROSCOPE;    
    Odometry_Rotation = GYROSCOPE.getYawRotation();   
    Module_Locking = (false);
    Path_Flipped = (false); 
    Current_Time = Timer.getFPGATimestamp();
    MODULES = List.of(
      Devices.FRONT_LEFT_MODULE,
      Devices.FRONT_RIGHT_MODULE,
      Devices.REAR_LEFT_MODULE,
      Devices.REAR_RIGHT_MODULE
    );    
    KINEMATICS = new SwerveDriveKinematics(
      new Translation2d( (DrivebaseConstants.Measurements.ROBOT_WIDTH_METERS)  / (2), 
                         (DrivebaseConstants.Measurements.ROBOT_LENGTH_METERS) / (2)),
      new Translation2d( (DrivebaseConstants.Measurements.ROBOT_WIDTH_METERS)  / (2),
                        -(DrivebaseConstants.Measurements.ROBOT_LENGTH_METERS) / (2)),
      new Translation2d(-(DrivebaseConstants.Measurements.ROBOT_WIDTH_METERS)  / (2),
                         (DrivebaseConstants.Measurements.ROBOT_LENGTH_METERS) / (2)),
      new Translation2d(-(DrivebaseConstants.Measurements.ROBOT_WIDTH_METERS)  / (2),
                        -(DrivebaseConstants.Measurements.ROBOT_LENGTH_METERS) / (2)));
    POSE_ESTIMATOR = new SwerveDrivePoseEstimator(
      KINEMATICS, 
      GYROSCOPE.getYawRotation(),
      getModulePositions(),   
      Odometry_Pose
    );
    AutoBuilder.configureHolonomic(
      DrivebaseSubsystem::getDrivebasePose,
      DrivebaseSubsystem::setDrivebasePose, 
      () -> KINEMATICS.toChassisSpeeds(getModuleMeasurements()), 
      DrivebaseSubsystem::set, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(
          Measurements.ROBOT_TRANSLATION_KP,
          Measurements.ROBOT_TRANSLATION_KI,
          Measurements.ROBOT_TRANSLATION_KP), 
        new PIDConstants(
          Measurements.ROBOT_ROTATIONAL_KP,
          Measurements.ROBOT_ROTATIONAL_KI,
          Measurements.ROBOT_ROTATIONAL_KD), 
        Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY, 
        Measurements.ROBOT_RADIUS_METERS, 
        new ReplanningConfig(
          (true),
          (true)
        )), 
      () -> Path_Flipped,
      Instance);
      Pathfinding.setPathfinder(new LocalADStarAK());
      PathPlannerLogging.setLogActivePathCallback(
        (ActivePath) -> {
          Logger.recordOutput(("Drivebase/Trajectory"), ActivePath.toArray(new Pose2d[ActivePath.size()]));
      });
      PathPlannerLogging.setLogTargetPoseCallback(
        (TargetPose) -> {
          Logger.recordOutput(("Drive/Setpoint"), TargetPose);
      });
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  public void periodic() {
    Objects.ODOMETRY_LOCKER.lock();
    MODULES.forEach(Module::update);
    GYROSCOPE.update();    
    if (DriverStation.isDisabled()) {
      MODULES.forEach(Module::stop);
    }
    Objects.ODOMETRY_LOCKER.unlock();
    AtomicInteger DeltaCount = new AtomicInteger(
      GYROSCOPE.getConnected()? 
        GYROSCOPE.getOdometryYawRotations().length: 
        Integer.MAX_VALUE
    );
    MODULES.forEach((Module) -> 
      DeltaCount.set(Math.min(DeltaCount.get(), Module.getPositionDeltas().length)
    ));
    IntStream.range((0), DeltaCount.get()).forEachOrdered((DeltaIndex) -> {
      SwerveModulePosition[] WheelDeltas = MODULES
        .stream()
        .map((Module) -> 
          Module.getPositionDeltas()[DeltaIndex])
      .toArray(SwerveModulePosition[]::new);
      var TwistDelta = KINEMATICS.toTwist2d(WheelDeltas);
      if(GYROSCOPE.getConnected()) {
        Rotation2d GyroRotationDelta = GYROSCOPE.getOdometryYawRotations()[DeltaIndex];
        TwistDelta = new Twist2d(TwistDelta.dx, TwistDelta.dy, GyroRotationDelta.minus(Odometry_Rotation).getRadians());
        Odometry_Rotation = GyroRotationDelta;
      }
      Odometry_Pose = Odometry_Pose.exp(TwistDelta);    
      var LocalizedTime = Timer.getFPGATimestamp();  
      POSE_ESTIMATOR.updateWithTime(
        LocalizedTime,
        Odometry_Rotation,
        WheelDeltas
      );
      POSE_ESTIMATOR.addVisionMeasurement(
        Odometry_Pose,
        LocalizedTime,
        (null)         //TODO: Act like Odometry Estimation is an vision pose to add it?
      );
      POSE_ESTIMATOR.addVisionMeasurement(
        (null),        //TODO: Pull Vision Measurements
        Timer.getFPGATimestamp(), 
        (null)         //TODO: Standard Deviations from Vision
      );
    });
  }

  /**
   * Calculates the discretization timestep, {@code dt}, at this current time based on the FPGA clock.
   * @return Double representation of the time passed between now and the last timestep.
   */
  private static synchronized Double discretize() {
    var DiscretizationTimestep = (0.0);
    if (Current_Time.equals((0.0))) {
      DiscretizationTimestep = ((1.0) / (50.0));
    } else {
      var MeasuredTime = Timer.getFPGATimestamp();
      DiscretizationTimestep = MeasuredTime - Current_Time;
      Current_Time = MeasuredTime;
    }    
    return DiscretizationTimestep;
  }

  /**
   * Closes this instance and all held resources immediately.
   */
  public synchronized void close() {
    POSE_ESTIMATOR.resetPosition(
      GYROSCOPE.getYawRotation(),
      getModulePositions(),
      new Pose2d()
    );
  }
  // --------------------------------------------------------------[Internal]---------------------------------------------------------------//

  public enum OrientationMode {
    OBJECT_ORIENTED,    
    ROBOT_ORIENTED,
    FIELD_ORIENTED,
  }
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Drives the robot provided a chassis speeds demand
   * @param Demand Chassis speeds object which represents the demand speeds of the drivebase
   */
  public static synchronized void set(final ChassisSpeeds Demand) {
    var ReferenceSpeeds = ChassisSpeeds.discretize(Demand, discretize());
    var ReferenceStates = KINEMATICS.toSwerveModuleStates(ReferenceSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      ReferenceStates, 
      ReferenceSpeeds, 
      Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
      Measurements.ROBOT_MAXIMUM_LINEAR_VELOCITY,
      Measurements.ROBOT_MAXIMUM_ANGULAR_VELOCITY);
    ReferenceStates = MODULES
    .stream()
    .map(
      (Module) -> Module.set(new SwerveModuleState())
    ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Drives the robot provided translation and rotational demands
   * @param Translation Demand translation in two-dimensional space
   * @param Rotation    Demand rotation in two-dimensional space
   * @param Mode        Type of demand being made
   * @param OpenLoop    Whether or not the demands 
   */
  public static synchronized void set(final Translation2d Translation, final Rotation2d Rotation, final OrientationMode Mode, final Boolean OpenLoop) {
    switch(Mode) {
      case OBJECT_ORIENTED:
        //TODO: Object Orientation
        break;      
      case ROBOT_ORIENTED:
        set(new ChassisSpeeds(
          Translation.getX(), 
          Translation.getY(), 
          Rotation.getRadians()));      
        break;
      case FIELD_ORIENTED:
        set(ChassisSpeeds.fromFieldRelativeSpeeds(
          Translation.getX(), 
          Translation.getY(), 
          Rotation.getRadians(), 
          GYROSCOPE.getYawRotation()));      
        break;
    }
  }

  /**
   * Stops all drivebase movement, if locking is enabled then all modules are 
   * reset into an 'X' orientation.
   */
  public static synchronized void set() {
    if(Module_Locking) {
      KINEMATICS.resetHeadings(MODULES.stream().map((Module) -> 
        Module.getMeasured().angle
      ).toArray(Rotation2d[]::new));  
    }
    set(new ChassisSpeeds());
  }

  /**
   * Mutates the current estimated pose of the robot
   * @param Pose Robot Pose in Meters
   */
  public static synchronized void setDrivebasePose(final Pose2d Pose) {
    Odometry_Pose = Pose;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the current position of the drivebase in space
   * @return Pose2d of Robot drivebase
   */
  public static Pose2d getDrivebasePose() {
    return POSE_ESTIMATOR.getEstimatedPosition();
  }

  /**
   * Provides the current un-optimized reference ,'set-point' state of all modules on the drivebase
   * @return Array of module states
   */
  public static SwerveModuleState[] getModuleReferences() {
    return MODULES.stream().sequential().map(
        (Module) ->  Module.getReference()
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current optimized reference ,'set-point' state of all modules on the drivebase
   * @return Array of module states
   */
  public static SwerveModuleState[] getModuleOptimized() {
    return MODULES.stream().sequential().map(
        (Module) ->  Module.getOptimized()
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current controller output state of all modules on the drivebase
   * @return Array of module states
   */
  public static SwerveModuleState[] getControllerOutputs() {
    return MODULES.stream().sequential().map(
        (Module) ->  Module.getOutput()
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current controller output state of all modules on the drivebase
   * @return Array of module states
   */
  public static SwerveModuleState[] getModuleMeasurements() {
    return MODULES.stream().sequential().map(
        (Module) ->  Module.getMeasured()
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current controller output positions of all modules on the drivebase
   * @return Array of module positions
   */
  public static SwerveModulePosition[] getModulePositions() {
    return MODULES.stream().map(
        (Module) ->  Module.getPosition()
      ).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Retrieves the existing instance of this static utility class
   * @return Utility class's instance
   */
   public static synchronized DrivebaseSubsystem getInstance() {
    if (java.util.Objects.isNull(Instance)) {
      Instance = new DrivebaseSubsystem();
    }
    return Instance;
  }
}
