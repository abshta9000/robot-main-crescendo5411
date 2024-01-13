// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.robotalons.crescendo.subsystems.drivebase.DrivebaseSubsystem;

import java.util.Set;
// ---------------------------------------------------------------[Constants]---------------------------------------------------------------//
/**
 *
 *
 * <h1>RobotConstants</h1>
 *
 * <p>Contains all robot-wide constants, does not contain subsystem specific constants.
 *
 * @see RobotContainer
 */
public final class Constants {
  // ------------------------------------------------------------[Internal]-------------------------------------------------------------//

  public static final class Subsystems {
      public static final Boolean IS_REAL_ROBOT = RobotBase.isReal();
      public static final Set<Subsystem> SUBSYSTEMS =  Set.of(
        DrivebaseSubsystem.getInstance()
      );
  }

  public static final class Logging {
      public static final String LOGGING_DEPOSIT_FOLDER = ("src\\main\\java\\main\\deploy\\logs");
      public static final Boolean LOGGING_TURBO_MODE = (false);
      public static final Boolean LOGGING_ENABLED = (false);
      public static final Boolean REPLAY_FROM_LOG = (false);
  }

  public static final class Ports {
      public static final Integer POWER_DISTRIBUTION_HUB = (0);
  }

  public static final class Simulation{
      public static final Double SIMULATION_LOOPPERIOD_SEC = .2;
  }
}
