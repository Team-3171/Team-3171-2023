package frc.team3171.controllers;

// Java Imports
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;

// FRC Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// REV Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Team 3171 Imports
import frc.robot.RobotProperties;
import frc.team3171.sensors.RevEncoder;
import frc.team3171.sensors.ThreadedPIDController;

/**
 * @author Mark Ebert
 */
public class Elevator implements RobotProperties {

    // Elevator Motors
    private final TalonFX elevatorMaster, elevatorSlave;
    private final TalonSRX leftAcuator, rightAcuator;
    private final CANSparkMax tiltyMagoo;
    private final RevEncoder elevatorEncoder;

    // PID Controller
    private final ThreadedPIDController elevatorPIDController;

    // Executor Service
    private final ExecutorService executorService;

    // Reentrant Locks
    private final ReentrantLock executorLock;

    // Atomic Booleans
    private final AtomicBoolean executorActive;

    /**
     * 
     */
    public Elevator() {
        // Init the master motor
        elevatorMaster = new TalonFX(ELEVATOR_ONE_CAN_ID, ELEVATOR_ONE_CAN_BUS);
        elevatorMaster.configFactoryDefault();
        elevatorMaster.setNeutralMode(NeutralMode.Brake);
        elevatorMaster.setInverted(ELEVATOR_INVERTED);
        elevatorMaster.setSelectedSensorPosition(0);

        // Init the slave motors
        elevatorSlave = new TalonFX(ELEVATOR_TWO_CAN_ID, ELEVATOR_TWO_CAN_BUS);
        elevatorSlave.configFactoryDefault();
        elevatorMaster.setNeutralMode(NeutralMode.Brake);
        elevatorSlave.follow(elevatorMaster);
        elevatorSlave.setInverted(InvertType.OpposeMaster);

        // Init the left acuator
        leftAcuator = new TalonSRX(LEFT_ACUATOR_CAN_ID);
        leftAcuator.configFactoryDefault();
        leftAcuator.setNeutralMode(NeutralMode.Coast);
        leftAcuator.setInverted(LEFT_ACUATOR_INVERTED);

        // Init the right acuator
        rightAcuator = new TalonSRX(RIGHT_ACUATOR_CAN_ID);
        rightAcuator.configFactoryDefault();
        rightAcuator.setNeutralMode(NeutralMode.Coast);
        rightAcuator.setInverted(RIGHT_ACUATOR_INVERTED);

        // Init the Tilty Magoo
        tiltyMagoo = new CANSparkMax(5, MotorType.kBrushless);

        // Elevator encoder init
        elevatorEncoder = new RevEncoder(ELEVATOR_ENCODER_CHANNEL);
        elevatorEncoder.reset();

        elevatorPIDController = new ThreadedPIDController(elevatorEncoder, ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD, ELEVATOR_PID_MIN, ELEVATOR_PID_MAX, false);
        elevatorPIDController.start(false);

        this.executorService = Executors.newSingleThreadExecutor();
        this.executorLock = new ReentrantLock(true);
        this.executorActive = new AtomicBoolean();
    }

    /**
     * 
     * @param speed
     */
    public void setElevatorSpeed(final double speed) {
        elevatorPIDController.disablePID();
        elevatorMaster.set(ControlMode.PercentOutput, speed);
    }

    /**
     * 
     * @param speed
     */
    public void setElevatorSpeed(final double speed, final double lowerBound, final double upperBound) {
        elevatorPIDController.disablePID();
        final double elevatorPosition = getElevatorPosition();
        if (speed < 0 && elevatorPosition <= lowerBound) {
            elevatorMaster.set(ControlMode.PercentOutput, 0);
        } else if (speed > 0 && elevatorPosition >= upperBound) {
            elevatorMaster.set(ControlMode.PercentOutput, 0);
        } else {
            elevatorMaster.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * 
     * @param position
     */
    public void setElevatorPosition(final double position, final double lowerBound, final double upperBound) {
        final double ElevatorPosition = getElevatorPosition();
        // final int winchTwoPosition = elevatorTwo.getEncoderValue();
        if (position < ElevatorPosition && ElevatorPosition <= lowerBound) {
            elevatorPIDController.disablePID();
            elevatorMaster.set(ControlMode.PercentOutput, 0);
        } else if (position > ElevatorPosition && ElevatorPosition >= upperBound) {
            elevatorPIDController.disablePID();
            elevatorMaster.set(ControlMode.PercentOutput, 0);
        } else {
            elevatorPIDController.enablePID();
            elevatorPIDController.updateSensorLockValueWithoutReset(position);
            elevatorMaster.set(ControlMode.PercentOutput, elevatorPIDController.getPIDValue());
        }
        SmartDashboard.putNumber("PID", elevatorPIDController.getPIDValue());
    }

    /**
     * 
     * @return The current encoder clicks.
     */
    public double getElevatorPosition() {
        return elevatorPIDController.getSensorValue();
    }

    public double getElevatorDesiredPoition() {
        return elevatorPIDController.getSensorLockValue();
    }

    /**
     * Sets the acuator to the given speed for the specified duration.
     * 
     * @param duration
     *            The duration that the acuator will run for before stopping.
     */
    public void setAcuatorSpeed(final double speed, final double duration) {
        try {
            executorLock.lock();
            if (executorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + duration;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (DriverStation.isDisabled()) {
                                break;
                            }
                            leftAcuator.set(ControlMode.PercentOutput, speed);
                            rightAcuator.set(ControlMode.PercentOutput, speed);
                            Timer.delay(.02);
                        }
                        leftAcuator.set(ControlMode.PercentOutput, 0);
                        rightAcuator.set(ControlMode.PercentOutput, 0);
                    } finally {
                        executorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    /**
     * Sets the acuator to the given speed for the specified duration.
     * 
     * @param timeout
     *            The timeout that the acuator will not exeed.
     */
    public void extendAcuatorToSensor(final double speed, final DigitalInput sensor, final double timeout) {
        try {
            executorLock.lock();
            if (executorActive.compareAndSet(false, true)) {
                executorService.execute(() -> {
                    try {
                        final double endTime = Timer.getFPGATimestamp() + timeout;
                        while (Timer.getFPGATimestamp() <= endTime) {
                            if (!sensor.get() || DriverStation.isDisabled()) {
                                break;
                            }
                            leftAcuator.set(ControlMode.PercentOutput, speed);
                            rightAcuator.set(ControlMode.PercentOutput, speed);
                            Timer.delay(.02);
                        }
                        leftAcuator.set(ControlMode.PercentOutput, 0);
                        rightAcuator.set(ControlMode.PercentOutput, 0);
                    } finally {
                        executorActive.set(false);
                    }
                });
            }
        } finally {
            executorLock.unlock();
        }
    }

    public void setAcuatorSpeed(final double speed) {
        if (!executorActive.get()) {
            leftAcuator.set(ControlMode.PercentOutput, speed);
            rightAcuator.set(ControlMode.PercentOutput, speed);
        }
    }

    public void setTitlyMagooSpeed(final double speed) {
        tiltyMagoo.set(speed);
    }

    /**
     * 
     */
    public void disable() {
        elevatorPIDController.disablePID();
        elevatorMaster.set(ControlMode.Disabled, 0);
        leftAcuator.set(ControlMode.Disabled, 0);
        rightAcuator.set(ControlMode.Disabled, 0);
        tiltyMagoo.disable();
    }

}
