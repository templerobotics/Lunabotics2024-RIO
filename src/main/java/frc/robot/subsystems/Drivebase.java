package frc.robot.subsystems;

import static frc.robot.Constants.DrivebaseConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.custom.LunaSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.custom.LunaSparkMax.Presets;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


import java.beans.Encoder;
import java.util.HashMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


public class Drivebase extends SubsystemBase {
    // Motor Initializations
    private final LunaSparkMax m_leftFront;
    private final LunaSparkMax m_leftRear;
    private final LunaSparkMax m_rightFront;
    private final LunaSparkMax m_rightRear;

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
	private HashMap<String, NetworkTableEntry> pidNTEntries = new HashMap<String, NetworkTableEntry>();
	private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();
	private SparkMaxPIDController[] pidControllers;

    private static SendableChooser<InputScaling> scalingChooser = new SendableChooser<>();

    

    public Drivebase() {
        m_leftFront = new LunaSparkMax(LEFT_FRONT_CAN_ID, MotorType.kBrushless, Presets.kDrivebase);
        m_leftRear = new LunaSparkMax(LEFT_REAR_CAN_ID, MotorType.kBrushless, Presets.kDrivebase);
        m_rightFront = new LunaSparkMax(RIGHT_FRONT_CAN_ID, MotorType.kBrushless, Presets.kDrivebase);
        m_rightRear = new LunaSparkMax(RIGHT_REAR_CAN_ID, MotorType.kBrushless, Presets.kDrivebase);

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
    }
    
    @Override
    public void periodic() {
        // Called once per scheduler run 
        reportSensors();
        if (ENABLE_TEST_DASHBOARDS) checkPIDGains();
    }

    private void addShuffleboardEntry(String key, String label, double defaultValue, int row, int col) {
        shuffleboardEntries.put(key, Shuffleboard.getTab("Competition")
                .add(label, defaultValue)
                .withSize(1, 1)
                .withPosition(row, col)
                .getEntry());
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
}


