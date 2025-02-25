package frc.team3171.drive;

// CTRE Imports
import com.ctre.phoenix.sensors.CANCoder;

/**
 * @author Mark Ebert
 */
public class SwerveUnitConfig {

    private final int slewMotor_CAN_ID, driveMotor_CAN_ID, absoluteEncoder_CAN_ID;
    private final boolean slewMotor_Inverted, driveMotor_Inverted;
    private final String canbus;

    /**
     * Constructor
     * 
     * @param slewMotor_CAN_ID
     *            The CAN ID of the {@link MotorController} used
     *            to control the slew motor.
     * @param driveMotor_CAN_ID
     *            The CAN ID of the {@link MotorController} used
     *            to control the drive motor.
     * @param absoluteEncoder_CAN_ID
     *            The CAN ID of the {@link CANCoder} used to tell
     *            the positiion of the wheel.
     * @param slewMotor_Inverted
     *            Whether or not the direction of the slew motor
     *            needs to be inverted.
     * @param driveMotor_Inverted
     *            Whether or not the direction of the drive motor
     *            needs to be inverted.
     * @param canbus
     *            The {@link String} of the CAN bus that the
     *            devices are located on. All three devices of
     *            the unit should be on the same bus.
     */
    public SwerveUnitConfig(int slewMotor_CAN_ID, int driveMotor_CAN_ID, int absoluteEncoder_CAN_ID,
            boolean slewMotor_Inverted, boolean driveMotor_Inverted, String canbus) {
        this.slewMotor_CAN_ID = slewMotor_CAN_ID;
        this.driveMotor_CAN_ID = driveMotor_CAN_ID;
        this.absoluteEncoder_CAN_ID = absoluteEncoder_CAN_ID;
        this.slewMotor_Inverted = slewMotor_Inverted;
        this.driveMotor_Inverted = driveMotor_Inverted;
        this.canbus = canbus;
    }

    /**
     * Constructor
     * 
     * @param slewMotor_CAN_ID
     *            The CAN ID of the {@link MotorController} used
     *            to control the slew motor.
     * @param driveMotor_CAN_ID
     *            The CAN ID of the {@link MotorController} used
     *            to control the drive motor.
     * @param absoluteEncoder_CAN_ID
     *            The CAN ID of the {@link CANCoder} used to tell
     *            the positiion of the wheel.
     * @param slewMotor_Inverted
     *            Whether or not the direction of the slew motor
     *            needs to be inverted.
     * @param driveMotor_Inverted
     *            Whether or not the direction of the drive motor
     *            needs to be inverted.
     */
    public SwerveUnitConfig(int slewMotor_CAN_ID, int driveMotor_CAN_ID, int absoluteEncoder_CAN_ID,
            boolean slewMotor_Inverted, boolean driveMotor_Inverted) {
        this(slewMotor_CAN_ID, driveMotor_CAN_ID, absoluteEncoder_CAN_ID, slewMotor_Inverted, driveMotor_Inverted, "");
    }

    public int getSlewMotor_CAN_ID() {
        return slewMotor_CAN_ID;
    }

    public int getDriveMotor_CAN_ID() {
        return driveMotor_CAN_ID;
    }

    public int getAbsoluteEncoder_CAN_ID() {
        return absoluteEncoder_CAN_ID;
    }

    public boolean isSlewMotor_Inverted() {
        return slewMotor_Inverted;
    }

    public boolean isDriveMotor_Inverted() {
        return driveMotor_Inverted;
    }

    public String getCanbus() {
        return canbus;
    }

}
