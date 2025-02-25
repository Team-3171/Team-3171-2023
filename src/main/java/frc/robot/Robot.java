// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Java Imports
import java.util.concurrent.ConcurrentLinkedQueue;

// FRC Imports
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

// CTRE Imports
import com.ctre.phoenix.sensors.WPI_Pigeon2;

// Team 3171 Imports
import frc.team3171.drive.SwerveDrive;
import frc.team3171.pnuematics.DoublePistonController;
import frc.team3171.sensors.ThreadedPIDController;
import frc.team3171.sensors.Limelight;
import frc.team3171.sensors.Pigeon2GravityVector;
import frc.team3171.HelperFunctions;
import frc.team3171.auton.AutonRecorder;
import frc.team3171.auton.AutonRecorderData;
import frc.team3171.auton.XboxControllerState;
import frc.team3171.controllers.Elevator;
import static frc.team3171.HelperFunctions.Normalize_Gryo_Value;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements RobotProperties {

  // Controllers
  private XboxController driveController, operatorController;

  // Drive Objects
  private SwerveDrive swerveDrive;
  private WPI_Pigeon2 gyro;
  private Pigeon2GravityVector gravityVector;
  private ThreadedPIDController gyroPIDController, balancingPIDController;
  // private ThreadedPIDController limelightOnePIDController;

  // Elevator Objects
  private Elevator elevator;
  private DoublePistonController pickupTilt, pickup;
  private Compressor compressor;
  private DigitalInput elevatorSensor;
  private Limelight limelightOne;
  // private Limelight limelightTwo;
  private AddressableLED ledStrip;
  private AddressableLEDBuffer rgbData = new AddressableLEDBuffer(60);

  // Auton Recorder
  private AutonRecorder autonRecorder;
  private ConcurrentLinkedQueue<AutonRecorderData> autonPlaybackQueue;
  private AutonRecorderData playbackData;
  private double autonStartTime;
  private boolean saveNewAuton;

  // Selected Auton String
  private boolean selectedAutonType;
  private String selectedAutonMode;

  // Shuffleboard Choosers
  private SendableChooser<Boolean> autonTypeChooser, fieldOrientationChooser;
  private SendableChooser<String> autonModeChooser;

  // Global Variables
  private boolean fieldOrientationChosen;
  private double elevatorDesiredPosition;
  private int rainbowFirstPixelHue = 0;
  private boolean rainbowFlipped = false;

  // Edge Triggers
  private boolean zeroEdgeTrigger, pickupEdgeTrigger, pickupTiltEdgeTrigger;

  @Override
  public void robotInit() {
    // Controllers Init
    driveController = new XboxController(0);
    operatorController = new XboxController(1);

    // Drive Controller Init
    swerveDrive = new SwerveDrive(lr_Unit_Config, lf_Unit_Config, rf_Unit_Config, rr_Unit_Config);

    // Elevator Controller Init
    elevator = new Elevator();

    // Pneumatics Init
    pickup = new DoublePistonController(PCM_CAN_ID, PneumaticsModuleType.REVPH, PICKUP_FORWARD_CHANNEL, PICKUP_REVERSE_CHANNEL, PICKUP_INVERTED);
    pickupTilt = new DoublePistonController(PCM_CAN_ID, PneumaticsModuleType.REVPH, PICKUPTILT_FORWARD_CHANNEL, PICKUPTILT_REVERSE_CHANNEL,
        PICKUPTILT_INVERTED);
    compressor = new Compressor(PCM_CAN_ID, PneumaticsModuleType.REVPH);
    compressor.enableAnalog(MIN_PRESSURE, MAX_PRESSURE);

    // Sensors
    gyro = new WPI_Pigeon2(GYRO_CAN_ID, GYRO_CAN_BUS);
    gyro.reset();
    gravityVector = new Pigeon2GravityVector(gyro);
    elevatorSensor = new DigitalInput(ELEVATOR_SENSOR_CHANNEL);
    limelightOne = new Limelight("limelight-one");
    // limelightTwo = new Limelight("limelight-two");
    limelightOne.turnLightOff();
    limelightOne.turnTrackingOff();
    // limelightTwo.turnLightOff();
    // limelightTwo.turnTrackingOff();

    // PID Controllers
    gyroPIDController = new ThreadedPIDController(gyro, GYRO_KP, GYRO_KI, GYRO_KD, GYRO_MIN, GYRO_MAX);
    // limelightOnePIDController = new ThreadedPIDController(limelightOne, LIMELIGHT_KP, LIMELIGHT_KI, LIMELIGHT_KD,
    // LIMELIGHT_MIN, LIMELIGHT_MAX, true);
    balancingPIDController = new ThreadedPIDController(gravityVector, DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_PID_MIN, DRIVE_PID_MAX, false);
    gyroPIDController.start();
    // limelightOnePIDController.start(true);
    balancingPIDController.start(true);

    // Auton Recorder init
    autonRecorder = new AutonRecorder();
    autonPlaybackQueue = new ConcurrentLinkedQueue<>();
    playbackData = null;
    saveNewAuton = false;

    // Auton Type init
    selectedAutonType = false;
    autonTypeChooser = new SendableChooser<>();
    autonTypeChooser.setDefaultOption("Playback Auton", false);
    autonTypeChooser.addOption("Record Auton", true);
    SmartDashboard.putData("Auton Type", autonTypeChooser);

    // Field Orientation Chooser
    fieldOrientationChooser = new SendableChooser<>();
    fieldOrientationChooser.setDefaultOption("Pick an option", null);
    fieldOrientationChooser.addOption("0\u00B0", false);
    fieldOrientationChooser.addOption("180\u00B0", true);
    SmartDashboard.putData("Field Orientation Chooser", fieldOrientationChooser);
    SmartDashboard.putBoolean("Flipped", false);

    // Auton Modes init
    selectedAutonMode = DEFAULT_AUTON;
    autonModeChooser = new SendableChooser<>();
    autonModeChooser.setDefaultOption(DEFAULT_AUTON, DEFAULT_AUTON);
    for (final String autonMode : AUTON_OPTIONS) {
      autonModeChooser.addOption(autonMode, autonMode);
    }
    SmartDashboard.putData("Auton Modes", autonModeChooser);

    // Global Variable Init
    fieldOrientationChosen = false;
    elevatorDesiredPosition = 0;

    // Edge Trigger Init
    zeroEdgeTrigger = false;
    pickupEdgeTrigger = false;
    pickupTiltEdgeTrigger = false;

    // LED Setup
    rgbData = new AddressableLEDBuffer(60);
    for (int i = 0; i < 60; i++) {
      rgbData.setRGB(i, 0, 200, 0);
    }
    ledStrip = new AddressableLED(0);
    ledStrip.setLength(rgbData.getLength());
    ledStrip.setData(rgbData);
    ledStrip.start();

    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
    // Gyro Value
    final double gyroValue = Normalize_Gryo_Value(gyro.getAngle());

    // Field Orientation Chooser
    final Boolean fieldOrientationBoolean = fieldOrientationChooser.getSelected();
    // Until a valid option is choosen leave the gyro orientation alone
    if (fieldOrientationBoolean != null && !fieldOrientationChosen) {
      // Prevents the field orientation from being changed until a reboot
      fieldOrientationChosen = true;
      // If the selected option is true then flip the orientation 180 degrees
      if (fieldOrientationBoolean.booleanValue()) {
        gyro.setYaw(Normalize_Gryo_Value(gyroValue + 180));
        SmartDashboard.putBoolean("Flipped", true);
      } else {
        // Else don't flip the field orientation
        SmartDashboard.putBoolean("Flipped", false);
      }
    }

    // Driver Controller Info
    double leftStickX, leftStickY, rightStickX, leftStickAngle, leftStickMagnitude, fieldCorrectedAngle;
    if (driveController.isConnected()) {
      // Get the controller values
      leftStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveController.getLeftX());
      leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -driveController.getLeftY());
      rightStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveController.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

      // Calculate the left stick angle and magnitude
      if (leftStickX != 0 || leftStickY != 0) {
        leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
        leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
        if (leftStickMagnitude > 1.0) {
          leftStickMagnitude = 1;
        }
      } else {
        leftStickAngle = 0;
        leftStickMagnitude = 0;
      }
      fieldCorrectedAngle = Normalize_Gryo_Value(leftStickAngle - gyroValue);
    } else {
      leftStickX = 0;
      leftStickY = 0;
      leftStickAngle = 0;
      leftStickMagnitude = 0;
      rightStickX = 0;
      fieldCorrectedAngle = 0;
    }

    // Put the values on Shuffleboard
    SmartDashboard.putBoolean("Elevator", !elevatorSensor.get());
    SmartDashboard.putString("Elevator Position", String.format("%.3f", elevator.getElevatorPosition()));
    SmartDashboard.putString("Elevator Lock", String.format("%.3f", elevator.getElevatorDesiredPoition()));
    SmartDashboard.putString("Air Pressure", String.format("%.2f PSI", compressor.getPressure()));
    SmartDashboard.putString("Gyro", String.format("%.2f\u00B0", gyroValue));
    if (DEBUG) {
      // Operator Controller Values
      final double elevatorSpeed = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -operatorController.getLeftY(), -MAX_ELEVATOR_SPEED,
          MAX_ELEVATOR_SPEED);
      final double acuatorSpeed = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE,
          operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis());
      SmartDashboard.putString("Left Stick X", String.format("%.2f", acuatorSpeed));
      SmartDashboard.putString("Left Stick Y", String.format("%.2f", leftStickY));
      SmartDashboard.putString("Right Stick X", String.format("%.2f", rightStickX));
      SmartDashboard.putString("Left Stick Angle", String.format("%.2f\u00B0", leftStickAngle));
      SmartDashboard.putString("Left Stick Velocity", String.format("%.2f", leftStickMagnitude));
      SmartDashboard.putString("Field Adjusted Angle", String.format("%.2f\u00B0", fieldCorrectedAngle));
      SmartDashboard.putString("Elevator Speed", String.format("%.2f", elevatorSpeed));
      swerveDrive.SmartDashboard();
    }
    SmartDashboard.putString("Tilt", String.format("%.3f", balancingPIDController.getSensorValue()));
    SmartDashboard.putString("Balancing PID", String.format("%.3f", balancingPIDController.getPIDValue()));

    // RGB Effects
    final double elevatorPosition = elevator.getElevatorPosition();
    int maxLEDIndex = (int) HelperFunctions.Map(elevatorPosition, ELEVATOR_LOWER_BOUND, ELEVATOR_UPPER_BOUND, 0, 59);
    maxLEDIndex = maxLEDIndex < 0 ? 0 : maxLEDIndex >= rgbData.getLength() ? rgbData.getLength() - 1 : maxLEDIndex;
    if (DriverStation.isDisabled()) {
      for (int i = 0; i < rgbData.getLength(); i++) {
        rgbData.setRGB(i, 0, 200, 0);
      }
    } else {
      for (int i = 0; i < rgbData.getLength(); i++) {
        if (i < maxLEDIndex) {
          rgbData.setHSV(i, (rainbowFirstPixelHue + (i * 180 / maxLEDIndex)) % 180, 255, 128);
        } else if (DriverStation.getAlliance() == Alliance.Blue) {
          rgbData.setRGB(i, 0, 0, 200);
        } else {
          rgbData.setRGB(i, 200, 0, 0);
        }
      }
      rainbowFirstPixelHue = rainbowFlipped ? rainbowFirstPixelHue - 3 : rainbowFirstPixelHue + 3;
      rainbowFlipped = rainbowFirstPixelHue >= 180 ? true : rainbowFirstPixelHue <= 0 ? false : rainbowFlipped;
    }
    ledStrip.setData(rgbData);

    // Tracking Info
    // SmartDashboard.putString("LL1", String.format("H:%.2f\u00B0 | V:%.2f\u00B0",
    // limelightOne.getTargetHorizontalOffset(), limelightOne.getTargetVerticalOffset()));
    // SmartDashboard.putString("LL2", String.format("H:%.2f | V:%.2f", limelightTwo.getTargetHorizontalOffset(),
    // limelightTwo.getTargetVerticalOffset()));

    // Calibrate Swerve Drive
    final boolean zeroTrigger = driveController.getBackButton() && driveController.getStartButton() && isDisabled();
    if (zeroTrigger && !zeroEdgeTrigger) {
      // Zero the swerve units
      swerveDrive.zero();
      System.out.println("Swerve Drive has been calibrated!");
    }
    zeroEdgeTrigger = zeroTrigger;
  }

  @Override
  public void autonomousInit() {
    // Update Auton Selected Mode and load the auton
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    if (selectedAutonType) {
      playbackData = null;
    } else {
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          disabledInit();
          playbackData = null;
          break;
        default:
          AutonRecorder.loadFromFile(autonPlaybackQueue, selectedAutonMode);
          playbackData = autonPlaybackQueue.poll();
          robotControlsInit();
          break;
      }
    }
    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    switch (selectedAutonMode) {
      case DEFAULT_AUTON:
        disabledPeriodic();
        break;
      default:
        // Plays the recorded auton if theres a valid next step, otherwise disables
        if (playbackData != null) {
          // Get the controller states
          final XboxControllerState driveControllerState = playbackData.getDriveControllerState();
          final XboxControllerState operatorControllerState = playbackData.getOperatorControllerState();

          // Robot drive controls
          robotControlsPeriodic(driveControllerState, operatorControllerState);

          // Checks for new data and when to switch to it
          if ((Timer.getFPGATimestamp() - autonStartTime) >= playbackData.getFPGATimestamp()) {
            playbackData = autonPlaybackQueue.poll();
          }
        } else {
          selectedAutonMode = DEFAULT_AUTON;
          disabledInit();
        }
        break;
    }
  }

  @Override
  public void teleopInit() {
    // Update Auton Selected Mode and reset the data recorder
    selectedAutonType = autonTypeChooser.getSelected();
    selectedAutonMode = autonModeChooser.getSelected();
    autonRecorder.clear();
    saveNewAuton = selectedAutonType;

    // Reset the robot controls
    robotControlsInit();

    // Update the autonStartTime
    autonStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void teleopPeriodic() {
    // Get the controller states
    final XboxControllerState driveControllerState = new XboxControllerState(driveController);
    final XboxControllerState operatorControllerState = new XboxControllerState(operatorController);

    // Robot drive controls
    robotControlsPeriodic(driveControllerState, operatorControllerState);

    // Auton Recording
    final double autonTimeStamp = Timer.getFPGATimestamp() - autonStartTime;
    if (saveNewAuton && autonTimeStamp <= 15) {
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          // Do Nothing
          break;
        default:
          // Adds the recorded data to the auton recorder, but only if the data is new
          autonRecorder.addNewData(new AutonRecorderData(autonTimeStamp, driveControllerState, operatorControllerState));
          break;
      }
    }
  }

  @Override
  public void disabledInit() {
    // Disable all controllers
    swerveDrive.disable();
    elevator.disable();
    pickupTilt.disable();
    pickup.disable();
    gyroPIDController.disablePID();
    // limelightOnePIDController.disablePID();
    balancingPIDController.disablePID();

    // Once auton recording is done, save the data to a file, if there is any
    if (saveNewAuton) {
      saveNewAuton = false;
      switch (selectedAutonMode) {
        case DEFAULT_AUTON:
          // Do Nothing
          break;
        default:
          autonRecorder.saveToFile(selectedAutonMode);
          break;
      }
    }
  }

  @Override
  public void disabledPeriodic() {
    // Do Nothing
  }

  @Override
  public void testInit() {
    // Do Nothing
  }

  @Override
  public void testPeriodic() {
    // Do Nothing
  }

  private void robotControlsInit() {
    // Reset the drive controller
    swerveDrive.driveInit();
    gyroPIDController.enablePID();
    gyroPIDController.updateSensorLockValue();
    balancingPIDController.disablePID();
    limelightOne.turnTrackingOff();
    limelightOne.turnLightOff();
    // limelightOnePIDController.disablePID();

    // Reset the elevator pneumatics
    pickup.retract();
    pickupTilt.retract();

    // Reset the global variables
    elevatorDesiredPosition = 0;

    // Reset edge triggers
    pickupEdgeTrigger = false;
    pickupTiltEdgeTrigger = false;
  }

  private void robotControlsPeriodic(final XboxControllerState driveControllerState, final XboxControllerState operatorControllerState) {
    // Gyro Value
    final double gyroValue = Normalize_Gryo_Value(gyro.getAngle());

    // Get the needed joystick values after calculating the deadzones
    final double leftStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getLeftX());
    final double leftStickY = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -driveControllerState.getLeftY());
    final double rightStickX = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, driveControllerState.getRightX(), -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

    // Calculate the left stick angle and magnitude
    final double leftStickAngle;
    double leftStickMagnitude;
    if (leftStickX != 0 || leftStickY != 0) {
      leftStickAngle = Normalize_Gryo_Value(Math.toDegrees(Math.atan2(leftStickX, leftStickY)));
      leftStickMagnitude = Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2));
      if (leftStickMagnitude > 1.0) {
        leftStickMagnitude = 1;
      }
    } else {
      leftStickAngle = 0;
      leftStickMagnitude = 0;
    }

    // Calculate the field corrected drive angle
    final double fieldCorrectedAngle = Normalize_Gryo_Value(leftStickAngle - gyroValue);

    // Operator Controller Values
    final double elevatorSpeed = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, -operatorControllerState.getLeftY(), MIN_ELEVATOR_SPEED,
        MAX_ELEVATOR_SPEED);
    final double acuatorSpeed = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE,
        operatorControllerState.getRightTriggerAxis() - operatorControllerState.getLeftTriggerAxis());
    final double tiltyMagooSpeed = HelperFunctions.Deadzone_With_Map(JOYSTICK_DEADZONE, operatorControllerState.getRightY(), -MAX_TILTY_MAGOO_SPEED,
        MAX_TILTY_MAGOO_SPEED);

    // Drive Controls
    final boolean slowMode = driveControllerState.getLeftBumper() || driveControllerState.getRightBumper();
    // final boolean poleTracking = driveControllerState.getLeftTriggerAxis() != 0;
    final boolean autoBalance = driveControllerState.getRightTriggerAxis() != 0;
    if (rightStickX != 0) {
      // Manual turning
      gyroPIDController.disablePID();
      balancingPIDController.disablePID();
      // limelightOne.turnTrackingOff();
      // limelightOne.turnLightOff();
      // limelightOnePIDController.disablePID();
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, rightStickX, slowMode);
    } /*
       * else if (poleTracking) {
       * // Target the pole using the limelight-one
       * balancingPIDController.disablePID();
       * limelightOne.turnTrackingOn();
       * limelightOne.turnLightOn();
       * if (limelightOne.hasTarget()) {
       * // Lock onto the pole if there is a valid target
       * gyroPIDController.disablePID();
       * //limelightOnePIDController.enablePID();
       * //limelightOnePIDController.updateSensorLockValueWithoutReset(2.5);
       * swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, limelightOnePIDController.getPIDValue(), slowMode);
       * } else {
       * // Gyro lock while there is no target
       * gyroPIDController.enablePID();
       * limelightOnePIDController.disablePID();
       * swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, gyroPIDController.getPIDValue(), slowMode);
       * }
       * }
       */
    else if (autoBalance) {
      // Autobalancing YOLO
      gyroPIDController.enablePID();

      // limelightOne.turnTrackingOff();
      // limelightOne.turnLightOff();
      // limelightOnePIDController.disablePID();
      if (HelperFunctions.Deadzone(.1825, balancingPIDController.getSensorValue()) == 0) {
        balancingPIDController.disablePID();
      } else {
        balancingPIDController.enablePID();
      }
      if (gyroValue > -90 && gyroValue < 90) {
        gyroPIDController.updateSensorLockValueWithoutReset(0);
        swerveDrive.drive(Normalize_Gryo_Value(-gyroValue), -balancingPIDController.getPIDValue(), gyroPIDController.getPIDValue(), slowMode);
      } else {
        gyroPIDController.updateSensorLockValueWithoutReset(180);
        swerveDrive.drive(Normalize_Gryo_Value(-gyroValue), balancingPIDController.getPIDValue(), gyroPIDController.getPIDValue(), slowMode);
      }
    } else {
      // Normal gyro locking
      gyroPIDController.enablePID();
      balancingPIDController.disablePID();
      // limelightOne.turnTrackingOff();
      // limelightOne.turnLightOff();
      // limelightOnePIDController.disablePID();
      // Quick Turning
      if (driveControllerState.getPOV() != -1) {
        gyroPIDController.updateSensorLockValueWithoutReset(Normalize_Gryo_Value(driveControllerState.getPOV()));
      } else if (driveControllerState.getYButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(0);
      } else if (driveControllerState.getBButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(90);
      } else if (driveControllerState.getAButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(180);
      } else if (driveControllerState.getXButton()) {
        gyroPIDController.updateSensorLockValueWithoutReset(-90);
      }
      swerveDrive.drive(fieldCorrectedAngle, leftStickMagnitude, gyroPIDController.getPIDValue(), slowMode);
    }

    // Elevator Height Controls
    if (elevatorSpeed != 0) {
      // Update desired elevator position to current position
      elevatorDesiredPosition = elevator.getElevatorPosition();
      // Manually set the elevator motors speed
      elevator.setElevatorSpeed(elevatorSpeed, ELEVATOR_LOWER_BOUND, elevatorSensor.get() ? ELEVATOR_VERTICAL_BOUND : ELEVATOR_UPPER_BOUND);
    } else {
      // If X button then human player, else if Y button then max extend
      if (operatorControllerState.getXButton()) {
        elevatorDesiredPosition = ELEVATOR_LOADER_STATION;
      } else if (operatorControllerState.getYButton()) {
        elevatorDesiredPosition = ELEVATOR_MAX_EXTEND;
      } else if (operatorControllerState.getPOV() == 180) {
        elevatorDesiredPosition = 0;
      }
      // Moved the elevator to the desired position with position restrictions
      elevator.setElevatorPosition(
          (elevatorDesiredPosition > ELEVATOR_VERTICAL_BOUND && elevatorSensor.get()) ? ELEVATOR_VERTICAL_BOUND : elevatorDesiredPosition,
          ELEVATOR_LOWER_BOUND, ELEVATOR_UPPER_BOUND);
    }

    // Elevator Tilt Controls
    elevator.setAcuatorSpeed(acuatorSpeed);

    // Pickup Controls
    if (operatorControllerState.getAButton() && !pickupEdgeTrigger) {
      pickup.toggle();
    }
    pickupEdgeTrigger = operatorControllerState.getAButton();

    // Pickup Tilt Controls
    if (elevator.getElevatorPosition() > ELEVATOR_LOWER_BOUND) {
      pickupTilt.extend();
    } else if (operatorControllerState.getBButton() && !pickupTiltEdgeTrigger && elevator.getElevatorPosition() < ELEVATOR_LOWER_BOUND) {
      pickupTilt.toggle();
    }
    pickupTiltEdgeTrigger = operatorControllerState.getBButton();

    // Tilty Magoo Controls
    elevator.setTitlyMagooSpeed(tiltyMagooSpeed);
  }

}
