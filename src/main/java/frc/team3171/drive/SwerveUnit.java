package frc.team3171.drive;

// Java Imports
import java.util.function.DoubleSupplier;

// CTRE Imports
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.sensors.ThreadedPIDController;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * @author Mark Ebert
 */
public class SwerveUnit implements DoubleSupplier, RobotProperties {

    // Motor Controllers
    private final FRCTalonFX driveMotor, slewMotor;

    // CANCoder
    private final CANCoder absoluteEncoder;

    // PID Controller
    private final ThreadedPIDController slewPIDController;

    // Global Variables
    private double startingAngle;

    /**
     * Constructor
     * 
     * @param driveInverted
     *            The config settings for the swerve unit.
     */
    public SwerveUnit(final SwerveUnitConfig swerveUnitConfig) {
        // Init the drive motor
        driveMotor = new FRCTalonFX(swerveUnitConfig.getDriveMotor_CAN_ID(), swerveUnitConfig.getCanbus());
        driveMotor.setInverted(swerveUnitConfig.isDriveMotor_Inverted());

        // Init the slew motor
        slewMotor = new FRCTalonFX(swerveUnitConfig.getSlewMotor_CAN_ID(), swerveUnitConfig.getCanbus());
        slewMotor.setInverted(swerveUnitConfig.isSlewMotor_Inverted());

        // Init the absolute position encoder used for the slew angle
        absoluteEncoder = new CANCoder(swerveUnitConfig.getAbsoluteEncoder_CAN_ID(), swerveUnitConfig.getCanbus());
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        // Init the gyro PID controller
        slewPIDController = new ThreadedPIDController(this, SLEW_KP, SLEW_KI, SLEW_KD, SLEW_PID_MIN, SLEW_PID_MAX, true);
        slewPIDController.start(true);

        // Init the global variables
        startingAngle = 0;
    }

    /**
     * Sets the drive motor to the desired speed.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the drive motor to.
     */
    public void setDriveSpeed(final double speed) {
        /*
         * Sets the speed of the master TalonFX, and therefore it's followers, to the
         * given value
         */
        driveMotor.set(speed);
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getDriveSpeed() {
        return driveMotor.getMotorOutputPercent();
    }

    /**
     * Sets whether or not the direction of the drive motor needs to be inverted.
     * 
     * @param inverted
     *            Whether or not the direction of the drive motor need to be
     *            inverted.
     */
    public void setDriveInverted(final boolean inverted) {
        /*
         * Sets whether or not the direction of the master TalonFX, and therefore it's
         * followers, need to be inverted
         */
        driveMotor.setInverted(inverted);
    }

    /**
     * Gets whether or not the direction of the drive motor is inverted.
     * 
     * @return True, if the drive motor is inverted, false otherwise.
     */
    public boolean getDriveInverted() {
        /*
         * Gets whether or not the direction of the master TalonFX, and therefore it's
         * followers, are inverted
         */
        return driveMotor.getInverted();
    }

    /**
     * Sets the slew motor to the desired angle.
     * 
     * @param angle
     *            The angle, from -180.0 to 180.0, to set the slew motor to.
     */
    public void updateSlewAngle(final double angle) {
        // Update the target angle of slew motor PID controller
        slewPIDController.updateSensorLockValueWithoutReset(angle);

        // Update Slew Motor Speed
        slewMotor.set(slewPIDController.getPIDValue());
    }

    /**
     * Updates the slew motors speed from the pid controller using the last updated target angle.
     */
    public void updateSlewAngle() {
        // Update Slew Motor Speed
        slewMotor.set(slewPIDController.getPIDValue());
    }

    /**
     * Sets the slew motor to the desired speed.
     * 
     * @param speed
     *            The speed, from -1.0 to 1.0, to set the slew motor to.
     */
    public void setSlewSpeed(final double speed) {
        slewMotor.set(speed);
    }

    /**
     * Returns the raw value of the {@link FRCTalonFX} integrated encoder. The
     * encoder has 2048 ticks per revolution.
     * 
     * @return The raw value of the {@link FRCTalonFX} integrated encoder.
     */
    public int getIntegratedEncoderValue() {
        return (int) driveMotor.getSelectedSensorPosition();
    }

    /**
     * Returns the velocity of the {@link FRCTalonFX} integrated
     * encoder. The encoder has 2048 ticks per revolution and the return units of
     * the velocity is in ticks per 100ms.
     * 
     * @return The velocity, in ticks per 100ms, of the {@link FRCTalonFX}
     *         integrated encoder.
     */
    public int getIntegratedEncoderVelocity() {
        return (int) driveMotor.getSelectedSensorVelocity();
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewAngle() {
        return Normalize_Gryo_Value(absoluteEncoder.getAbsolutePosition() - startingAngle);
    }

    /**
     * Returns the current position of the {@link CANCoder}.
     * 
     * @return The current position, in degrees, from -180 to 180.
     */
    public double getSlewTargetAngle() {
        return slewPIDController.getSensorLockValue();
    }

    /**
     * Returns the current velocity of the {@link CANCoder}.
     * 
     * @return The velocity, in degrees per second.
     */
    public double getSlewVelocity() {
        return absoluteEncoder.getVelocity();
    }

    public void enable() {
        slewPIDController.enablePID();
    }

    /**
     * Disables the drive and slew {@link TalonFX} motors.
     */
    public void disable() {
        driveMotor.disable();
        slewMotor.disable();
        slewPIDController.disablePID();
    }

    @Override
    public double getAsDouble() {
        return getSlewAngle();
    }

    public void zeroModule() {
        startingAngle = absoluteEncoder.getAbsolutePosition();
    }

    public double getSlewOffset() {
        return startingAngle;
    }

    public void setSlewOffset(final double slewOffset) {
        startingAngle = slewOffset;
    }

}