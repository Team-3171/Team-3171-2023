package frc.robot;

// Team 3171 Imports
import frc.team3171.drive.SwerveUnitConfig;

/**
 * @author Mark Ebert
 */
public interface RobotProperties {

        /** Debug Info **/
        public static final boolean DEBUG = true;

        /** Drive Variables **/
        public static final double JOYSTICK_DEADZONE = .08;
        public static final double SLOW_DRIVE_SPEED = .6, MAX_ROTATION_SPEED = .6, MIN_ELEVATOR_SPEED = -.2, MAX_ELEVATOR_SPEED = .35;
        public static final double MAX_TILTY_MAGOO_SPEED = .3;

        /** Elevator Variables **/
        public static final double ELEVATOR_LOWER_BOUND = 1, ELEVATOR_VERTICAL_BOUND = 5.3, ELEVATOR_UPPER_BOUND = 5.865;
        public static final double ELEVATOR_LOADER_STATION = 3.6, ELEVATOR_MAX_EXTEND = 5.84;

        /** Swerve Unit Configuration **/
        public static final SwerveUnitConfig lr_Unit_Config = new SwerveUnitConfig(11, 10, 18, false, false, "canivore");
        public static final SwerveUnitConfig lf_Unit_Config = new SwerveUnitConfig(13, 12, 19, false, false, "canivore");
        public static final SwerveUnitConfig rf_Unit_Config = new SwerveUnitConfig(15, 14, 20, false, false, "canivore");
        public static final SwerveUnitConfig rr_Unit_Config = new SwerveUnitConfig(17, 16, 21, false, false, "canivore");

        /** CAN ID Properties **/
        public static final int GYRO_CAN_ID = 22;
        public static final int ELEVATOR_ONE_CAN_ID = 24, ELEVATOR_TWO_CAN_ID = 23;
        public static final int LEFT_ACUATOR_CAN_ID = 3, RIGHT_ACUATOR_CAN_ID = 4;
        public static final int PCM_CAN_ID = 2;
        public static final int TILTY_MAGOO_CAN_ID = 5;

        /** Pneumatic Channels **/
        public static final int PICKUP_FORWARD_CHANNEL = 0, PICKUP_REVERSE_CHANNEL = 1;
        public static final int PICKUPTILT_FORWARD_CHANNEL = 4, PICKUPTILT_REVERSE_CHANNEL = 5;

        /** CAN BUS Properties **/
        public static final String GYRO_CAN_BUS = "canivore";
        public static final String ELEVATOR_ONE_CAN_BUS = "canivore", ELEVATOR_TWO_CAN_BUS = "canivore";

        /** Inversion Properties **/
        public static final boolean ELEVATOR_INVERTED = true;
        public static final boolean LEFT_ACUATOR_INVERTED = true, RIGHT_ACUATOR_INVERTED = true;
        public static final boolean PICKUP_INVERTED = false, PICKUPTILT_INVERTED = true;

        /** Compressor Properties **/
        public static final double MIN_PRESSURE = 95, MAX_PRESSURE = 110;

        /** Sensor Channels **/
        public static int ELEVATOR_SENSOR_CHANNEL = 0;
        public static int ELEVATOR_ENCODER_CHANNEL = 1;

        /** PID Properties **/
        public static final double LIMELIGHT_KP = -.0175, LIMELIGHT_KI = -.0022, LIMELIGHT_KD = -.0022, LIMELIGHT_MIN = -.5, LIMELIGHT_MAX = .5;
        public static final double ELEVATOR_KP = .6, ELEVATOR_KI = .1, ELEVATOR_KD = 0, ELEVATOR_KF = 0, ELEVATOR_PID_MIN = -.25, ELEVATOR_PID_MAX = .4;
        public static final double GYRO_KP = .013, GYRO_KI = .00075, GYRO_KD = .00075, GYRO_MIN = -.5, GYRO_MAX = .5;
        public static final double SLEW_KP = .008, SLEW_KI = .005, SLEW_KD = .005, SLEW_KF = 0, SLEW_PID_MIN = -.5, SLEW_PID_MAX = .5;
        public static final double DRIVE_KP = .7, DRIVE_KI = 0, DRIVE_KD = 0, DRIVE_KF = 0, DRIVE_PID_MIN = -.3, DRIVE_PID_MAX = .3;

        /** Auton Mode Constants **/
        public static final String DEFAULT_AUTON = "Disabled";

        /** Auton Modes **/
        public static final String[] AUTON_OPTIONS = {
                        "M CU BAL", "L CU CO", "R CU CO",
                        "No Bump Balance-Red", "No Bump Balance-Blue",
                        "No Bump-Red", "No Bump-Blue",
                        "Cube Taxi Balance","Blue 2-Piece","Red 2-Piece",
                        "Extra 1", "Extra 2", "Extra 3"
        };

}