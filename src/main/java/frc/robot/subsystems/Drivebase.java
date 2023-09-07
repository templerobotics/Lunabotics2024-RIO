package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.custom.LunaMathUtils;
import frc.robot.custom.LunaSparkMax;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import com.revrobotics.CANPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.custom.LunaSparkMax.Presets;
import frc.robot.Constants.DrivebaseConstants;

import java.util.HashMap;

public class Drivebase extends SubsystemBase {
    // Motors
    private final LunaSparkMax m_leftFront;
    private final LunaSparkMax m_rightFront;
    private final LunaSparkMax m_leftRear;
    private final LunaSparkMax m_rightRear;

    // PID Controllers
    private final SparkMaxPIDController p_leftFront;
    private final SparkMaxPIDController p_rightFront;
    private final SparkMaxPIDController p_leftRear;
    private final SparkMaxPIDController p_rightRear;

    // Encoders 
    private final RelativeEncoder e_leftFront;
    private final RelativeEncoder e_rightFront;
    private final RelativeEncoder e_leftRear;
    private final RelativeEncoder e_rightRear;

    // Network Tables
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable;    


    public static enum Encoders {
        leftFront, rightFront, leftRear, rightRear, leftAvg, rightAvg, allAvg
    }

    public static enum Motors {
        leftFront, rightFront, leftRear, rightRear
    }

    public static enum IMU {
        accelX, accelY, accelZ, gyroX, gyroY, gyroZ
    }

    public enum InputScaling {
		Linear, Cubic
	}

	public interface ScalarFunction {
		public double calculate(double input);
	}

    private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
	private HashMap<String, NetworkTableEntry> pidNTEntries = new HashMap<String, NetworkTableEntry>();
	private HashMap<String, NetworkTableEntry> shuffleboardEntries = new HashMap<String, NetworkTableEntry>();
	private SparkMaxPIDController[] pidControllers;

    public Drivebase() {
        // Motor Instantiation
        m_leftFront = new LunaSparkMax(DrivebaseConstants.LEFT_FRONT_CAN_ID, kBrushless, Presets.kDrivebase);
        m_leftRear = new LunaSparkMax(DrivebaseConstants.LEFT_REAR_CAN_ID, kBrushless, Presets.kDrivebase);
        m_rightFront = new LunaSparkMax(DrivebaseConstants.RIGHT_FRONT_CAN_ID, kBrushless, Presets.kDrivebase);
        m_rightRear = new LunaSparkMax(DrivebaseConstants.RIGHT_REAR_CAN_ID, kBrushless, Presets.kDrivebase);

        // Encoder Instantiation
        e_leftFront = m_leftFront.getEncoder();
		e_leftRear = m_leftRear.getEncoder();
		e_rightFront = m_rightFront.getEncoder();
		e_rightRear = m_rightRear.getEncoder();

        // PID Controller Instantiation
        p_leftFront = m_leftFront.getPIDController();
		p_leftRear = m_leftRear.getPIDController();
		p_rightFront = m_rightFront.getPIDController();
		p_rightRear = m_rightRear.getPIDController();
        pidControllers = new SparkMaxPIDController[] {p_leftFront, p_leftRear, p_rightFront, p_rightRear};

        // NetworkTable Instantiation
        networkTable = ntInstance.getTable("DrivebaseSubsystem");

        // Set DPP Values
		e_leftFront.setPositionConversionFactor(DrivebaseConstants.LEFT_FRONT_DPR);
		e_leftRear.setPositionConversionFactor(DrivebaseConstants.LEFT_REAR_DPR);
		e_rightFront.setPositionConversionFactor(DrivebaseConstants.RIGHT_FRONT_DPR);
		e_rightRear.setPositionConversionFactor(DrivebaseConstants.RIGHT_REAR_DPR);

    }


}
