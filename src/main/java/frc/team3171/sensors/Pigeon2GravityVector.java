package frc.team3171.sensors;

// Java Imports
import java.util.function.DoubleSupplier;

// FRC Imports
import com.ctre.phoenix.sensors.WPI_Pigeon2;

/**
 * @author Mark Ebert
 */
public class Pigeon2GravityVector implements DoubleSupplier {

    // Already initialized pigeon2
    private final WPI_Pigeon2 pigeon2;

    public Pigeon2GravityVector(final WPI_Pigeon2 pigeon2) {
        this.pigeon2 = pigeon2;
    }

    @Override
    public double getAsDouble() {
        final double[] gravityVector = new double[3];
        pigeon2.getGravityVector(gravityVector);
        return gravityVector[1];
    }

}
