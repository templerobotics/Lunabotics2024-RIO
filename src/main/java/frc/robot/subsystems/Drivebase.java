package frc.robot.subsystems;

import static frc.robot.Constants.DrivebaseConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.custom.LunaSparkMax.Presets;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.custom.LunaSparkTest;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.custom.LunaMathUtils;
import static frc.robot.Constants.GlobalConstants.*;

import java.util.HashMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class Drivebase extends SubsystemBase {
    // Motor Initializations
    private final LunaSparkTest m_leftFront;
    private final LunaSparkTest m_leftRear;
    private final LunaSparkTest m_rightFront;
    private final LunaSparkTest m_rightRear;

    // Encoder Initializations
    private final RelativeEncoder e_leftFront;
    private final RelativeEncoder e_leftRear;
    private final RelativeEncoder e_rightFront;
    private final RelativeEncoder e_rightRear;

    // PID Controllers Initialization
    private final SparkMaxPIDController p_leftFront;
    private final SparkMaxPIDController p_leftRear;
    private final SparkMaxPIDController p_rightFront;
    private final SparkMaxPIDController p_rightRear;

    // Motor enums
    public static enum Motors {
        leftFront, leftRear, rightFront, rightRear
    }

    // Encoder enums
    public static enum Encoders {
        leftFront, leftRear, rightFront, rightRear, leftAvg, rightAvg, overallAvg
    }

    public enum InputScaling {
		Linear, Cubic
	}

    public interface ScalarFunction {
		public double calculate(double input);
	}

    // Motor Controller Groups
    private final MotorControllerGroup leftGroup;
    private final MotorControllerGroup rightGroup;

    // NetworkTable Instantiation
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable;

    private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
	private HashMap<String, GenericEntry> pidNTEntries = new HashMap<String, GenericEntry>();
	private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();
	private SparkMaxPIDController[] pidControllers;

    private static SendableChooser<InputScaling> scalingChooser = new SendableChooser<>();

    

    public Drivebase() {
        m_leftFront = new LunaSparkTest(LEFT_FRONT_CAN_ID, MotorType.kBrushless, Presets.kDrivebase);
        m_leftRear = new LunaSparkTest(LEFT_REAR_CAN_ID, MotorType.kBrushless, Presets.kDrivebase);
        m_rightFront = new LunaSparkTest(RIGHT_FRONT_CAN_ID, MotorType.kBrushless, Presets.kDrivebase);
        m_rightRear = new LunaSparkTest(RIGHT_REAR_CAN_ID, MotorType.kBrushless, Presets.kDrivebase);

        e_leftFront = m_leftFront.getEncoder();
        e_leftRear = m_leftRear.getEncoder();
        e_rightFront = m_rightFront.getEncoder();
        e_rightRear = m_rightRear.getEncoder();

        p_leftFront = m_leftFront.getPIDController();
        p_leftRear = m_leftRear.getPIDController();
        p_rightFront = m_rightFront.getPIDController();
        p_rightRear = m_rightRear.getPIDController();

        m_leftFront.setInverted(INVERT_LEFT);
        m_leftRear.setInverted(INVERT_LEFT);
        m_rightFront.setInverted(INVERT_RIGHT);
        m_rightRear.setInverted(INVERT_RIGHT);

        networkTable = ntInstance.getTable("DrivebaseSubsystem");

        e_leftFront.setPositionConversionFactor(LEFT_FRONT_DPR);
        e_leftRear.setPositionConversionFactor(LEFT_REAR_DPR);
        e_rightFront.setPositionConversionFactor(RIGHT_FRONT_DPR);
        e_rightRear.setPosition(RIGHT_REAR_DPR);

        m_leftFront.burnFlash();
        m_leftRear.burnFlash();
        m_rightFront.burnFlash();
        m_rightRear.burnFlash();

        leftGroup = new MotorControllerGroup(m_leftFront, m_leftFront);
        rightGroup = new MotorControllerGroup(m_rightFront, m_rightRear);

        createDashboardData();

        if (ENABLE_TEST_DASHBOARDS) {
            reportInitialPID();
            pidConstants.put("drive-kP", PID_kP);
			pidConstants.put("drive-kI", PID_kI);
			pidConstants.put("drive-kD", PID_kD);
			pidConstants.put("drive-kIZ", PID_kIZ);
			pidConstants.put("drive-kFF", PID_kFF);
			pidConstants.put("drive-setpoint", 0.0);
        }
    }
    
    @Override
    public void periodic() {
        // Called once per scheduler run 
        reportSensors();
        if (ENABLE_TEST_DASHBOARDS) checkPIDGains();
    }   

    private void createDashboardData() {
		boolean unitFlag = APPLY_VELOCITY_SCALAR; // This solves a "cannot resolve import" error for whatever reason
		shuffleboardEntries.put("speed-units", Shuffleboard.getTab("Competition").add("Speed Units", unitFlag ? "m/s" : "RPM")
				.withSize(1, 1).withPosition(1, 0).getEntry());
		shuffleboardEntries.put("speed-lf", Shuffleboard.getTab("Competition").add("LF Speed", 0).withSize(1, 1).withPosition(0, 1).getEntry());
		shuffleboardEntries.put("speed-rf", Shuffleboard.getTab("Competition").add("RF Speed", 0).withSize(1, 1).withPosition(1, 1).getEntry());
		shuffleboardEntries.put("speed-lr", Shuffleboard.getTab("Competition").add("LR Speed", 0).withSize(1, 1).withPosition(0, 2).getEntry());
		shuffleboardEntries.put("speed-rr", Shuffleboard.getTab("Competition").add("RR Speed", 0).withSize(1, 1).withPosition(1, 2).getEntry());
		shuffleboardEntries.put("encoder-lf", Shuffleboard.getTab("Competition").add("LF Encoder", 0).withSize(1, 1).withPosition(0, 3).getEntry());
		shuffleboardEntries.put("encoder-rf", Shuffleboard.getTab("Competition").add("RF Encoder", 0).withSize(1, 1).withPosition(1, 3).getEntry());
		shuffleboardEntries.put("encoder-lr", Shuffleboard.getTab("Competition").add("LR Encoder", 0).withSize(1, 1).withPosition(0, 4).getEntry());
		shuffleboardEntries.put("encoder-rr", Shuffleboard.getTab("Competition").add("RR Encoder", 0).withSize(1, 1).withPosition(1, 4).getEntry());
		scalingChooser.setDefaultOption("Cubic", InputScaling.Cubic);
		scalingChooser.addOption("Linear", InputScaling.Linear);
		shuffleboardEntries.put("imu-moving", Shuffleboard.getTab("Competition").add("IMU Moving", false).withSize(1, 1).withPosition(14, 0).getEntry());
		shuffleboardEntries.put("imu-accel-x", Shuffleboard.getTab("Competition").add("IMU X-Accel", 0).withSize(1, 1).withPosition(14, 1).getEntry());
		shuffleboardEntries.put("imu-accel-y", Shuffleboard.getTab("Competition").add("IMU Y-Accel", 0).withSize(1, 1).withPosition(14, 2).getEntry());
		shuffleboardEntries.put("imu-accel-z", Shuffleboard.getTab("Competition").add("IMU Z-Accel", 0).withSize(1, 1).withPosition(14, 3).getEntry());
		shuffleboardEntries.put("imu-gyro-x", Shuffleboard.getTab("Competition").add("IMU X-Gyro", 0).withSize(1, 1).withPosition(15, 1).getEntry());
		shuffleboardEntries.put("imu-gyro-y", Shuffleboard.getTab("Competition").add("IMU Y-Gyro", 0).withSize(1, 1).withPosition(15, 2).getEntry());
		shuffleboardEntries.put("imu-gyro-z", Shuffleboard.getTab("Competition").add("IMU Z-Gyro", 0).withSize(1, 1).withPosition(15, 3).getEntry());
	}

    public void drive(double leftSpeed, double rightSpeed) {
        leftGroup.set(leftSpeed);
        rightGroup.set(rightSpeed);
    }

    public void setMotorSpeeds(double[] motorSpeeds) {
        if (motorSpeeds.length != 4) return;
        m_leftFront.set(motorSpeeds[0]);
        m_leftRear.set(motorSpeeds[1]);
        m_rightFront.set(motorSpeeds[2]);
        m_rightRear.set(motorSpeeds[3]);
    }

    public void setPIDSpeeds(double[] motorSpeeds) {
        if (motorSpeeds.length != 4) return; 
        p_leftFront.setReference(motorSpeeds[0], ControlType.kSmartVelocity);
        p_leftRear.setReference(motorSpeeds[1], ControlType.kSmartVelocity);
        p_rightFront.setReference(motorSpeeds[2], ControlType.kSmartVelocity);
        p_rightRear.setReference(motorSpeeds[3], ControlType.kSmartVelocity);
    }

    public void setMotorSpeeds(Motors motor, double speed) {
        switch (motor) {
            case leftFront:
                m_leftFront.set(speed);
                break;
            case leftRear:
                m_leftRear.set(speed);
                break;
            case rightFront:
                m_rightFront.set(speed);
                break;
            case rightRear:
                m_rightRear.set(speed);
                break;
        }
    }

    public void setPIDSpeed(Motors motor, double speed) {
        switch (motor) {
			case leftFront:
				p_leftFront.setReference(speed, ControlType.kSmartVelocity);
				break;

			case leftRear:
				p_leftRear.setReference(speed, ControlType.kSmartVelocity);
				break;

			case rightFront:
				p_rightFront.setReference(speed, ControlType.kSmartVelocity);
				break;

			case rightRear:
				p_rightRear.setReference(speed, ControlType.kSmartVelocity);
				break;
		}
    }

    public void stop() {
        leftGroup.stopMotor();
        rightGroup.stopMotor();
    }

    public double getEncoderValue(Encoders encoder) {
        switch (encoder) {
            case overallAvg:
                return (e_leftFront.getPosition() + e_leftRear.getPosition() + e_rightFront.getPosition()
						+ e_rightRear.getPosition()) / 4;
			case leftAvg:
				return (e_leftFront.getPosition() + e_leftRear.getPosition()) / 2;
			case rightAvg:
				return (e_rightFront.getPosition() + e_rightRear.getPosition()) / 2;
            case leftFront:
                return e_leftFront.getPosition();
            case leftRear:
                return e_leftRear.getPosition();
            case rightFront:
                return e_rightFront.getPosition();
            case rightRear:
                return e_rightRear.getPosition();

            default:
                return getEncoderValue(Encoders.overallAvg);     
        }
    }

    public double getMotorOutput(Motors motor) {
        switch (motor) {
            case leftFront:
                return m_leftFront.get();
            case leftRear:
                return m_leftRear.get();
            case rightFront:
                return m_rightFront.get();
            case rightRear:
                return m_rightRear.get();
            default:
                return 0;
        }
    }

    public double getMotorSpeed(Motors motor) {
        switch (motor) {
            case leftFront:
                return e_leftFront.getVelocity();
            case leftRear:
                return e_leftRear.getVelocity();
            case rightFront:
                return e_rightFront.getVelocity();
            case rightRear:
                return e_rightRear.getVelocity();
            default:
                return 0; 
        }
    }

    public double getMotorCurrent(Motors motor) {
        switch (motor) {
            case leftFront:
                return m_leftFront.getOutputCurrent();
            case leftRear:
                return m_leftRear.getOutputCurrent();
            case rightFront:
                return m_rightFront.getOutputCurrent(); 
            case rightRear: 
                return m_rightRear.getOutputCurrent();
            default:
                return 0;
        }
    }

    public void resetEncoders() {
        e_leftFront.setPosition(0);
        e_leftRear.setPosition(0);
        e_rightFront.setPosition(0);
        e_rightRear.setPosition(0);
    }

    public void resetEncoders(Motors motor) {
        switch (motor) {
            case leftFront: 
                e_leftFront.setPosition(0);
                break;
            case leftRear:
                e_leftRear.setPosition(0);
                break;
            case rightFront: 
                e_rightFront.setPosition(0);
                break;
            case rightRear:
                e_rightRear.setPosition(0);
                break;
        }
    }

    public ScalarFunction getInputScaling() {
        InputScaling selectedScaler = scalingChooser.getSelected();
        if (selectedScaler == InputScaling.Cubic) {
            return new ScalarFunction() {
                @Override
                public double calculate(double input) {
                    return (0.85 * Math.pow(input, 3) + (0.15 * input));
                }
            };
        } else {
            return new ScalarFunction() {
                @Override
                public double calculate(double input) {
                    return input;
                }
            };
        }
    }

    /*
    public void resetIMU() {
		navX.reset();
	}

	public void calibrateIMU() {
		navX.calibrate();
	}

	public boolean isIMUMoving() {
		return navX.isMoving();
	}

    public float getIMUData(IMU data) {
		switch (data) {
			case accelX:
				return navX.getAccelX();

			case accelY:
				return navX.getAccelY();

			case accelZ:
				return navX.getAccelZ();

			case gyroX:
				return navX.getGyroX();

			case gyroY:
				return navX.getGyroY();

			case gyroZ:
				return navX.getGyroZ();

			default:
				return 0;
		}
	}
    */

    private void reportInitialPID() {
		pidNTEntries.put("drive-kP", Shuffleboard.getTab("Drivebase PID").add("P Gain", PID_kP).getEntry());
		pidNTEntries.put("drive-kI", Shuffleboard.getTab("Drivebase PID").add("I Gain", PID_kI).getEntry());
		pidNTEntries.put("drive-kD", Shuffleboard.getTab("Drivebase PID").add("D Gain", PID_kD).getEntry());
		pidNTEntries.put("drive-kIZ", Shuffleboard.getTab("Drivebase PID").add("I Zone", PID_kIZ).getEntry());
		pidNTEntries.put("drive-kFF", Shuffleboard.getTab("Drivebase PID").add("Feed Forward", PID_kFF).getEntry());
		pidNTEntries.put("drive-setpoint", Shuffleboard.getTab("Drivebase PID").add("Setpoint", 0).getEntry());
		pidNTEntries.put("drive-lf-velocity", Shuffleboard.getTab("Drivebase PID").add("LF Velocity", 0).getEntry());
		pidNTEntries.put("drive-lr-velocity", Shuffleboard.getTab("Drivebase PID").add("LR Velocity", 0).getEntry());
		pidNTEntries.put("drive-rf-velocity", Shuffleboard.getTab("Drivebase PID").add("RF Velocity", 0).getEntry());
		pidNTEntries.put("drive-rr-velocity", Shuffleboard.getTab("Drivebase PID").add("RR Velocity", 0).getEntry());
	}

    private void checkPIDGains() {
        for (String pidConstant: pidConstants.keySet()) {
            double newVal = pidNTEntries.get(pidConstant).getDouble(pidConstants.get(pidConstant));
            if (newVal == pidConstants.get(pidConstant)) continue;
            String[] split = pidConstant.split("-", 2);
            for (SparkMaxPIDController pidController : pidControllers) {
                switch (split[1]) {
                    case "kP":
                        pidController.setP(newVal);
                        break;
                    case "kI":
                        pidController.setI(newVal);
                        break;
                    case "kD":
                        pidController.setD(newVal);
                        break;
                    case "kIZ":
                        pidController.setIZone(newVal);
                        break;
                    case "kFF":
                        pidController.setFF(newVal);
                        break;
                    case "setpoint":
                        pidController.setReference(newVal, ControlType.kVelocity);
                        break;
                    default:
                        break;
                }
            }
            pidConstants.put(pidConstant, newVal);
        }
        pidNTEntries.get("drive-lf-velocity").setDouble(e_leftFront.getVelocity());
		pidNTEntries.get("drive-lr-velocity").setDouble(e_leftRear.getVelocity());
		pidNTEntries.get("drive-rf-velocity").setDouble(e_rightFront.getVelocity());
		pidNTEntries.get("drive-rr-velocity").setDouble(e_rightRear.getVelocity());
    }

    private void reportSensors() {
		// Encoders
		shuffleboardEntries.get("encoder-lf").setDouble(LunaMathUtils.roundToPlace(e_leftFront.getPosition(), 2));
		shuffleboardEntries.get("encoder-lr").setDouble(LunaMathUtils.roundToPlace(e_leftRear.getPosition(), 2));
		shuffleboardEntries.get("encoder-rf").setDouble(LunaMathUtils.roundToPlace(e_rightFront.getPosition(), 2));
		shuffleboardEntries.get("encoder-rr").setDouble(LunaMathUtils.roundToPlace(e_rightRear.getPosition(), 2));

		// Motor Speed
		boolean unitFlag = APPLY_VELOCITY_SCALAR; // This solves a "cannot resolve import" error for whatever reason
		shuffleboardEntries.get("speed-units").setString(unitFlag ? "m/s" : "RPM");
		shuffleboardEntries.get("speed-lf").setDouble(LunaMathUtils.roundToPlace(e_leftFront.getVelocity(), 2));
		shuffleboardEntries.get("speed-lf").setDouble(LunaMathUtils.roundToPlace(e_leftRear.getVelocity(), 2));
		shuffleboardEntries.get("speed-lf").setDouble(LunaMathUtils.roundToPlace(e_rightFront.getVelocity(), 2));
		shuffleboardEntries.get("speed-lf").setDouble(LunaMathUtils.roundToPlace(e_rightRear.getVelocity(), 2));

		// navX IMU
        /*
		shuffleboardEntries.get("imu-accel-x").setNumber(LunaMathUtils.roundToPlace(navX.getAccelX(), 2));
		shuffleboardEntries.get("imu-accel-y").setNumber(LunaMathUtils.roundToPlace(navX.getAccelY(), 2));
		shuffleboardEntries.get("imu-accel-z").setNumber(LunaMathUtils.roundToPlace(navX.getAccelZ(), 2));
		shuffleboardEntries.get("imu-gyro-x").setNumber(LunaMathUtils.roundToPlace(navX.getGyroX(), 2));
		shuffleboardEntries.get("imu-gyro-y").setNumber(LunaMathUtils.roundToPlace(navX.getGyroY(), 2));
		shuffleboardEntries.get("imu-gyro-z").setNumber(LunaMathUtils.roundToPlace(navX.getGyroZ(), 2));
		shuffleboardEntries.get("imu-moving").setBoolean(navX.isMoving());
        */

		// Backend NetworkTable
		networkTable.getEntry("leftFrontPosition").setDouble(e_leftFront.getPosition());
		networkTable.getEntry("leftRearPosition").setDouble(e_leftRear.getPosition());
		networkTable.getEntry("rightFrontPosition").setDouble(e_rightFront.getPosition());
		networkTable.getEntry("rightRearPosition").setDouble(e_rightRear.getPosition());
		networkTable.getEntry("leftFrontVelocity").setDouble(e_leftFront.getVelocity());
		networkTable.getEntry("leftRearVelocity").setDouble(e_leftRear.getVelocity());
		networkTable.getEntry("rightFrontVelocity").setDouble(e_rightFront.getVelocity());
		networkTable.getEntry("rightRearVelocity").setDouble(e_rightRear.getVelocity());
		networkTable.getEntry("leftFrontCurrent").setDouble(m_leftFront.getOutputCurrent());
		networkTable.getEntry("leftRearCurrent").setDouble(m_leftRear.getOutputCurrent());
		networkTable.getEntry("rightFrontCurrent").setDouble(m_rightFront.getOutputCurrent());
		networkTable.getEntry("rightRearCurrent").setDouble(m_rightRear.getOutputCurrent());
        /*
		networkTable.getEntry("imuXAccel").setDouble(navX.getAccelX());
		networkTable.getEntry("imuYAccel").setDouble(navX.getAccelY());
		networkTable.getEntry("imuZAccel").setDouble(navX.getAccelZ());
		networkTable.getEntry("imuXGyro").setDouble(navX.getGyroX());
		networkTable.getEntry("imuYGyro").setDouble(navX.getGyroY());
		networkTable.getEntry("imuZGyro").setDouble(navX.getGyroZ());
        */
	}
}


