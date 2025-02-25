package frc.team3171.drive;

// Java Imports
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

// FRC Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Team 3171 Imports
import frc.robot.RobotProperties;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;
import static frc.team3171.HelperFunctions.Add_Two_Vectors;
import static frc.team3171.HelperFunctions.Return_Vector_With_Largest_Magnitude;
import static frc.team3171.HelperFunctions.Map;
import static frc.team3171.HelperFunctions.Get_Gyro_Displacement;

/**
 * @author Mark Ebert
 */
public class SwerveDrive implements RobotProperties {

    // Swerve Drive Units
    private SwerveUnit lrUnit, lfUnit, rfUnit, rrUnit;

    // Global Variables
    private volatile double lrAngle = 0, lfAngle = 0, rfAngle = 0, rrAngle = 0;

    public SwerveDrive(final SwerveUnitConfig lrUnitConfig, final SwerveUnitConfig lfUnitConfig, final SwerveUnitConfig rfUnitConfig,
            final SwerveUnitConfig rrUnitConfig) {
        this.lrUnit = new SwerveUnit(lrUnitConfig);
        this.lfUnit = new SwerveUnit(lfUnitConfig);
        this.rfUnit = new SwerveUnit(rfUnitConfig);
        this.rrUnit = new SwerveUnit(rrUnitConfig);

        loadSlewCalibration();
    }

    public void driveInit() {
        lrAngle = 0;
        lfAngle = 0;
        rfAngle = 0;
        rrAngle = 0;
        lrUnit.enable();
        lfUnit.enable();
        rfUnit.enable();
        rrUnit.enable();
    }

    public void drive(final double driveAngle, final double driveMagnitude, final double rotationalMagnitude, final boolean slowMode) {
        double lfMagnitude, lrMagnitude, rfMagnitude, rrMagnitude;
        if (driveMagnitude != 0 || rotationalMagnitude != 0) {
            // Create the initial vector
            final double[] driveVector = new double[] { driveAngle, driveMagnitude };

            // Perform the vector addition to get the resultant vectors
            final double[] lfResultantVector = Add_Two_Vectors(driveVector, new double[] { 45, rotationalMagnitude });
            final double[] lrResultantVector = Add_Two_Vectors(driveVector, new double[] { -45, rotationalMagnitude });
            final double[] rfResultantVector = Add_Two_Vectors(driveVector, new double[] { 135, rotationalMagnitude });
            final double[] rrResultantVector = Add_Two_Vectors(driveVector, new double[] { -135, rotationalMagnitude });

            // Update the angles
            lfAngle = lfResultantVector[0];
            lrAngle = lrResultantVector[0];
            rfAngle = rfResultantVector[0];
            rrAngle = rrResultantVector[0];

            // Find vector with the largest magnitude
            final double[] largestVector = Return_Vector_With_Largest_Magnitude(lfResultantVector, lrResultantVector, rfResultantVector,
                    rrResultantVector);

            // Scale the magnitudes if the largest vector exceeds 1.0
            if (largestVector[1] > 1.0) {
                // Update the magnitudes
                lfMagnitude = Map(lfResultantVector[1], 0, largestVector[1], 0, slowMode ? SLOW_DRIVE_SPEED : 1);
                lrMagnitude = Map(lrResultantVector[1], 0, largestVector[1], 0, slowMode ? SLOW_DRIVE_SPEED : 1);
                rfMagnitude = Map(rfResultantVector[1], 0, largestVector[1], 0, slowMode ? SLOW_DRIVE_SPEED : 1);
                rrMagnitude = Map(rrResultantVector[1], 0, largestVector[1], 0, slowMode ? SLOW_DRIVE_SPEED : 1);
            } else {
                // Update the magnitudes
                lfMagnitude = Map(lfResultantVector[1], 0, 1, 0, slowMode ? SLOW_DRIVE_SPEED : 1);
                lrMagnitude = Map(lrResultantVector[1], 0, 1, 0, slowMode ? SLOW_DRIVE_SPEED : 1);
                rfMagnitude = Map(rfResultantVector[1], 0, 1, 0, slowMode ? SLOW_DRIVE_SPEED : 1);
                rrMagnitude = Map(rrResultantVector[1], 0, 1, 0, slowMode ? SLOW_DRIVE_SPEED : 1);
            }
        } else {
            // Keep the current wheel angles but update the magnitudes
            lfMagnitude = 0;
            lrMagnitude = 0;
            rfMagnitude = 0;
            rrMagnitude = 0;
        }

        // Swerve Direction Optimization
        final double lfAngleDisplacement = Math.abs(Get_Gyro_Displacement(lfUnit.getSlewAngle(), lfAngle));
        final double lrAngleDisplacement = Math.abs(Get_Gyro_Displacement(lrUnit.getSlewAngle(), lrAngle));
        final double rfAngleDisplacement = Math.abs(Get_Gyro_Displacement(rfUnit.getSlewAngle(), rfAngle));
        final double rrAngleDisplacement = Math.abs(Get_Gyro_Displacement(rrUnit.getSlewAngle(), rrAngle));

        if (lfAngleDisplacement > 90) {
            lfAngle = Normalize_Gryo_Value(lfAngle + 180);
            lfMagnitude = -lfMagnitude;
        }
        if (lrAngleDisplacement > 90) {
            lrAngle = Normalize_Gryo_Value(lrAngle + 180);
            lrMagnitude = -lrMagnitude;
        }
        if (rfAngleDisplacement > 90) {
            rfAngle = Normalize_Gryo_Value(rfAngle + 180);
            rfMagnitude = -rfMagnitude;
        }
        if (rrAngleDisplacement > 90) {
            rrAngle = Normalize_Gryo_Value(rrAngle + 180);
            rrMagnitude = -rrMagnitude;
        }

        // Updates the slew motor angles
        lfUnit.updateSlewAngle(lfAngle);
        lrUnit.updateSlewAngle(lrAngle);
        rfUnit.updateSlewAngle(rfAngle);
        rrUnit.updateSlewAngle(rrAngle);

        // Set the drive motor speeds
        lrUnit.setDriveSpeed(lrMagnitude);
        lfUnit.setDriveSpeed(lfMagnitude);
        rfUnit.setDriveSpeed(rfMagnitude);
        rrUnit.setDriveSpeed(rrMagnitude);
    }

    public void disable() {
        lrAngle = 0;
        lfAngle = 0;
        rfAngle = 0;
        rrAngle = 0;
        lrUnit.disable();
        lfUnit.disable();
        rfUnit.disable();
        rrUnit.disable();
    }

    public void zero() {
        lrUnit.zeroModule();
        lfUnit.zeroModule();
        rfUnit.zeroModule();
        rrUnit.zeroModule();
        final double lrSlewOffset = lrUnit.getSlewOffset();
        final double lfSlewOffset = lfUnit.getSlewOffset();
        final double rfSlewOffset = rfUnit.getSlewOffset();
        final double rrSlewOffset = rrUnit.getSlewOffset();
        saveSlewCalibration(String.format("%.4f,%.4f,%.4f,%.4f;\n", lrSlewOffset, lfSlewOffset, rfSlewOffset, rrSlewOffset));
    }

    /**
     * Saves the current data collected by the auton recorder to the specified file
     * path and clears the AutonRecorder.
     * 
     * @param autonFileName
     *            The path to the file to save the auton data to.
     */
    public synchronized void saveSlewCalibration(final String slewOffsets) {
        try {
            File calibrationFile = new File(String.format("/home/lvuser/%s.txt", "slewcalibration"));
            if (calibrationFile.exists()) {
                calibrationFile.delete();
            }
            calibrationFile.createNewFile();
            BufferedWriter writer = new BufferedWriter(new FileWriter(calibrationFile));
            writer.write(slewOffsets);
            // Flush the data to the file
            writer.flush();
            writer.close();
            System.out.println("Calibration Successfully Saved!");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * 
     * @param autonFileName
     * @return
     */
    public synchronized void loadSlewCalibration() {
        try {
            BufferedReader reader = new BufferedReader(new FileReader(String.format("/home/lvuser/%s.txt", "slewcalibration")));
            String dataString = reader.readLine();

            // Parse the string
            dataString = dataString.trim();
            if (dataString.endsWith(";")) {
                dataString = dataString.substring(0, dataString.length() - 1);
                final String[] data = dataString.split(",");
                if (data.length == 4) {
                    final double lrSlewOffset = Double.parseDouble(data[0]);
                    final double lfSlewOffset = Double.parseDouble(data[1]);
                    final double rfSlewOffset = Double.parseDouble(data[2]);
                    final double rrSlewOffset = Double.parseDouble(data[3]);

                    lrUnit.setSlewOffset(lrSlewOffset);
                    lfUnit.setSlewOffset(lfSlewOffset);
                    rfUnit.setSlewOffset(rfSlewOffset);
                    rrUnit.setSlewOffset(rrSlewOffset);
                }
            }
            reader.close();
            System.out.println("Calibration Successfully Loaded!");
        } catch (IOException e) {
            System.err.printf("The Calibration File %s.txt could not be found!\n", "slewcalibration");
        }
    }

    public void SmartDashboard() {
        // Update Shuffleboard
        SmartDashboard.putString("LF Speed", String.format("%.2f", lfUnit.getDriveSpeed()));
        SmartDashboard.putString("LR Speed", String.format("%.2f", lrUnit.getDriveSpeed()));
        SmartDashboard.putString("RF Speed", String.format("%.2f", rfUnit.getDriveSpeed()));
        SmartDashboard.putString("RR Speed", String.format("%.2f", rrUnit.getDriveSpeed()));

        SmartDashboard.putString("LR Slew Angle", String.format("%.2f | %.2f", lrUnit.getSlewAngle(), lrUnit.getSlewTargetAngle()));
        SmartDashboard.putString("LF Slew Angle", String.format("%.2f | %.2f", lfUnit.getSlewAngle(), lfUnit.getSlewTargetAngle()));
        SmartDashboard.putString("RF Slew Angle", String.format("%.2f | %.2f", rfUnit.getSlewAngle(), rfUnit.getSlewTargetAngle()));
        SmartDashboard.putString("RR Slew Angle", String.format("%.2f | %.2f", rrUnit.getSlewAngle(), rrUnit.getSlewTargetAngle()));
    }

}
