// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package org.robotalons.crescendo;

// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import edu.wpi.first.wpilibj.RobotBase;
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
    }

    public static final class Logging {
        public static final String LOGGING_DEPOSIT_FOLDER = ("src\\main\\java\\main\\deploy\\logs");
        public static final Boolean LOGGING_TURBO_MODE = (false);
        public static final Boolean LOGGING_ENABLED = (false);
        public static final Boolean REPLAY_FROM_LOG = (false);
    }

    public static final class Ports {
        public static final Integer POWER_DISTRIBUTION_HUB = (0);
        public static final Integer PATHPLANNER_SERVER = (6969);
    }
}
