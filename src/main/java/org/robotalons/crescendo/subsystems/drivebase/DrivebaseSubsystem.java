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

import org.littletonrobotics.junction.AutoLogOutput;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Devices;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Measurements;
import org.robotalons.crescendo.subsystems.drivebase.Constants.Objects;
import org.robotalons.lib.motion.DrivebaseModule;
import org.robotalons.lib.motion.Gyroscope;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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
@SuppressWarnings("unused")
public final class DrivebaseSubsystem extends SubsystemBase implements Closeable {
  // --------------------------------------------------------------[Constants]--------------------------------------------------------------//
  private static final SwerveDrivePoseEstimator POSE_ESTIMATOR;
  private static final SwerveDriveKinematics KINEMATICS;  
  private static final List<DrivebaseModule> MODULES;  
  private static final Gyroscope GYROSCOPE;  
  // ---------------------------------------------------------------[Fields]----------------------------------------------------------------//
  private static DrivebaseSubsystem Instance;
  private static Rotation2d Odometry_Rotation;
  private static Pose2d Odometry_Pose;  
  private static Boolean Path_Flipped;  
  private static Double Current_Time;
  // ------------------------------------------------------------[Constructors]-------------------------------------------------------------//
  /**
   * Drivebase Subsystem Constructor.
   */
  private DrivebaseSubsystem() {} static {
    Instance = new DrivebaseSubsystem();
    Odometry_Pose = new Pose2d();
    GYROSCOPE = Devices.GYROSCOPE;    
    Odometry_Rotation = GYROSCOPE.getYawRotation();   
    Path_Flipped = (true); 
    Current_Time = Timer.getFPGATimestamp();
    MODULES = List.of(
      Devices.FRONT_LEFT_MODULE,
      Devices.FRONT_RIGHT_MODULE,
      Devices.REAR_LEFT_MODULE,
      Devices.REAR_RIGHT_MODULE
    );    
    KINEMATICS = new SwerveDriveKinematics(
      new Translation2d( (Constants.Measurements.ROBOT_WIDTH_METERS)  / (2), 
                         (Constants.Measurements.ROBOT_LENGTH_METERS) / (2)),
      new Translation2d( (Constants.Measurements.ROBOT_WIDTH_METERS)  / (2),
                        -(Constants.Measurements.ROBOT_LENGTH_METERS) / (2)),
      new Translation2d(-(Constants.Measurements.ROBOT_WIDTH_METERS)  / (2),
                         (Constants.Measurements.ROBOT_LENGTH_METERS) / (2)),
      new Translation2d(-(Constants.Measurements.ROBOT_WIDTH_METERS)  / (2),
                        -(Constants.Measurements.ROBOT_LENGTH_METERS) / (2)));
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
      Pathfinding.setPathfinder(null);
  }
  // ---------------------------------------------------------------[Methods]---------------------------------------------------------------//
  public synchronized void periodic() {
    Objects.ODOMETRY_LOCKER.lock();
    MODULES.forEach(DrivebaseModule::update);
    GYROSCOPE.update();    
    if (DriverStation.isDisabled()) {
      MODULES.forEach(DrivebaseModule::stop);
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

  public synchronized void simulationPeriodic() {
    periodic();
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
  // --------------------------------------------------------------[Mutators]---------------------------------------------------------------//
  /**
   * Mutates the goal states of the drivebase
   * @param Demand Chassis speeds object which represents the demand speeds of the drivebase
   */
  public static void set(final ChassisSpeeds Demand) {
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
   * Mutates the current estimated pose of the robot
   * @param Pose Robot Pose in Meters
   */
  public static void setDrivebasePose(final Pose2d Pose) {
    Odometry_Pose = Pose;
  }
  // --------------------------------------------------------------[Accessors]--------------------------------------------------------------//
  /**
   * Provides the current position of the drivebase in space
   * @return Pose2d of Robot drivebase
   */
  @AutoLogOutput(key = "Drivebase/Pose")
  public static Pose2d getDrivebasePose() {
    return POSE_ESTIMATOR.getEstimatedPosition();
  }

  /**
   * Provides the current un-optimized reference ,'set-point' state of all modules on the drivebase
   * @return Array of module states
   */
  @AutoLogOutput(key = "Drivebase/References")
  public static SwerveModuleState[] getModuleReferences() {
    return MODULES.stream().sequential().map(
        (Module) ->  Module.getReference()
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current optimized reference ,'set-point' state of all modules on the drivebase
   * @return Array of module states
   */
  @AutoLogOutput(key = "Drivebase/Optimized")
  public static SwerveModuleState[] getModuleOptimized() {
    return MODULES.stream().sequential().map(
        (Module) ->  Module.getOptimized()
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current controller output state of all modules on the drivebase
   * @return Array of module states
   */
  @AutoLogOutput(key = "Drivebase/Outputs")
  public static SwerveModuleState[] getControllerOutputs() {
    return MODULES.stream().sequential().map(
        (Module) ->  Module.getOutput()
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current controller output state of all modules on the drivebase
   * @return Array of module states
   */
  @AutoLogOutput(key = "Drivebase/Measurements")
  public static SwerveModuleState[] getModuleMeasurements() {
    return MODULES.stream().sequential().map(
        (Module) ->  Module.getMeasured()
      ).toArray(SwerveModuleState[]::new);
  }

  /**
   * Provides the current controller output positions of all modules on the drivebase
   * @return Array of module positions
   */
  @AutoLogOutput(key = "Drivebase/Positions")
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
