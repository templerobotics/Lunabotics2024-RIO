package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import com.revrobotics.SparkMaxAnalogSensor;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import frc.robot.custom.LunaMathUtils;
import static frc.robot.Constants.DiggingConstants.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstants.RobotSide;
import frc.robot.custom.LunaSparkMax;
import static frc.robot.Constants.GlobalConstants.*;
import frc.robot.custom.LunaMathUtils;

import java.util.HashMap;

public class Dumping extends SubsystemBase {
    public enum LinearActuatorState {
		Unknown, Raised, Lowered, TravelingUp, TravelingDown, Commanded;
	}

    private final LunaSparkMax m_linear1;
    private final SparkMaxAnalogSensor a_linear1;
    private final SparkMaxPIDController p_linear;

    // NetworkTable Instantiation
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable;

    private boolean linearInitialized = false;
    private LinearActuatorState linearState = LinearActuatorState.Unknown;
    private Runnable linearRunnable;
    private Notifier linearNotifier;

    private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
	private HashMap<String, GenericEntry> pidNTEntries = new HashMap<String, GenericEntry>();
	private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();

    private SparkMaxPIDController[] pidControllers;

    public Dumping() {
        m_linear1 = new LunaSparkMax(LINEAR_1_CAN_ID, MotorType.kBrushed);
        a_linear1 = m_linear1.getAnalogSensor();
        p_linear = m_linear1.getPIDController();

        m_linear1.burnFlash();

        networkTable = ntInstance.getTable("DumpingSubsystem");

        createDashBoardData();
        linearStop();

        if (ENABLE_TEST_DASHBOARDS){
            reportInitialPID();
            pidConstants.put("dumping-kP", LINEAR_kP);
            pidConstants.put("dumping-kI", LINEAR_kI);
            pidConstants.put("dumping-kD", LINEAR_kD);
            pidConstants.put("dumping-kIZ", LINEAR_kIZ);
            pidConstants.put("dumping-kFF", LINEAR_kFF);
            pidConstants.put("dumping-setpoint", 0.0);
        }
    }

    private void createDashBoardData() {
        shuffleboardEntries.put("dumping-init", Shuffleboard.getTab("Competition").add("Linear Init", linearInitialized)
			.withSize(1, 1).withPosition(3, 0).getEntry());
		shuffleboardEntries.put("dumping-state", Shuffleboard.getTab("Competition")
			.add("Linear State", linearState.toString()).withSize(1, 1).withPosition(3, 1).getEntry());
        shuffleboardEntries.put("dumping-position",
			Shuffleboard.getTab("Competition").add("Dumping Position", 0).withSize(1, 1).withPosition(3, 2).getEntry());
        shuffleboardEntries.put("position-command",
            Shuffleboard.getTab("Competition").getLayout("LACommand", BuiltInLayouts.kList).withSize(1, 2)
                .withPosition(3, 4).add("Position", 0.0).getEntry());
    }

    @Override
    public void periodic() {
        // This will be called once per scheduler run
        reportSensors();
        checkLimits();
        checkPIDGains();
    }

    private void linearUp() {
        /*
        if (a_linear1.getPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)
				|| linearState == LinearActuatorState.Raised) {
                    m_linear1.set(-1);
                    return;
                }
			        
        linearState = LinearActuatorState.TravelingUp;
        m_linear1.set(-1);*/

        double test = LunaMathUtils.scaleBetween(a_linear1.getPosition(), 0, 5, 0, 1);
        System.out.println(test);
        if (test >= 0.6 || linearState == LinearActuatorState.Lowered)
            return;
        linearState = LinearActuatorState.TravelingDown;
        m_linear1.set(-1);
    }       

    private void linearDown() {
        double test = LunaMathUtils.scaleBetween(a_linear1.getPosition(), 0, 5, 0, 1);
        System.out.println(test);
		if (a_linear1.getPosition() <= LINEAR_MIN_TRAVEL
				|| linearState == LinearActuatorState.Lowered)
			return;
		linearState = LinearActuatorState.TravelingDown;
        m_linear1.set(1);
    }

    private void linearStop() {
        m_linear1.set(0);
    }

    public void linearActuatorInitStart() {
        linearDown();
    }

    public double linearActuatorGetRawPotentiometer() {
        return a_linear1.getVoltage();
    }

    public void linearActuatorInitEnd() {
		linearStop();
		linearRunnable = new LinearActuatorLimits();
		linearNotifier = new Notifier(linearRunnable);
		linearNotifier.startPeriodic(0.005);
		linearInitialized = true;
	}

    public boolean isLinearActuatorInitialized() {
        return linearInitialized;
    }

    public void linearActuator(LinearActuatorState state) {
		if (state == LinearActuatorState.Raised) {
			linearUp();
		} else if (state == LinearActuatorState.Lowered) {
			linearDown();
		}
	}

    public LinearActuatorState linearActuatorState() {
        return linearState;
    }


    public double getDashboardCommandedPosition() {
		return MathUtil.clamp(shuffleboardEntries.get("position-command").getDouble(LINEAR_MIN_TRAVEL),
				LINEAR_MIN_TRAVEL, LINEAR_MAX_TRAVEL);
	}

    public double getLinearPosition() {
        return a_linear1.getPosition();
    }

    public void commandDown(){
		linearDown();
		linearState = LinearActuatorState.Commanded;
	}

	public void commandUp(){
		linearUp();
		linearState = LinearActuatorState.Commanded;
	}

	public void commandStop(){
		linearStop();
	}

    private void checkPIDGains() {
		for (String pidConstant : pidConstants.keySet()) {
			double newVal = pidNTEntries.get(pidConstant).getDouble(pidConstants.get(pidConstant));
			if (newVal == pidConstants.get(pidConstant))
				continue;
			// System.out.println("LS: " + pidConstant + " value updated: " + newVal);
			String[] split = pidConstant.split("-", 2);
                switch (split[1]) {
                    case "kP":
                        p_linear.setP(newVal);
                        break;
                    case "kI":
                        p_linear.setI(newVal);
                        break;
                    case "kD":
                        p_linear.setD(newVal);
                        break;
                    case "kIZ":
                        p_linear.setIZone(newVal);
                        break;
                    case "kFF":
                        p_linear.setFF(newVal);
                        break;
                    case "setpoint":
                        p_linear.setReference(newVal, ControlType.kVelocity);
                        break;
                    default:
                        break;
                }
			pidConstants.put(pidConstant, newVal);
		}
		pidNTEntries.get("dumping-velocity").setDouble(a_linear1.getPosition());
	}

    private void reportSensors() {
		shuffleboardEntries.get("dumping-init").setBoolean(linearInitialized);
		shuffleboardEntries.get("dumping-state").setString(linearState.toString());
        shuffleboardEntries.get("dumping-position").setDouble(LunaMathUtils.roundToPlace(a_linear1.getPosition(), 3));
		networkTable.getEntry("linearInitialized").setBoolean(linearInitialized);
        networkTable.getEntry("DumpingPosition").setDouble(a_linear1.getPosition());
		networkTable.getEntry("linearState").setString(linearState.toString());
	}

    private void reportInitialPID(){
        pidNTEntries.put("dumping-kP", Shuffleboard.getTab("Dumping PID").add("Dumping P Gain", BELT_kP).getEntry());
		pidNTEntries.put("dumping-kI", Shuffleboard.getTab("Dumping PID").add("Dumping I Gain", BELT_kI).getEntry());
		pidNTEntries.put("dumping-kD", Shuffleboard.getTab("Dumping PID").add("Dumping D Gain", BELT_kD).getEntry());
		pidNTEntries.put("dumping-kIZ", Shuffleboard.getTab("Dumping PID").add("Dumping I Zone", BELT_kIZ).getEntry());
		pidNTEntries.put("dumping-kFF", Shuffleboard.getTab("Dumping PID").add("Dumping FF", BELT_kFF).getEntry());
		pidNTEntries.put("dumping-setpoint", Shuffleboard.getTab("Dumping PID").add("Dumping Setpoint", 0).getEntry());
		pidNTEntries.put("dumping-velocity", Shuffleboard.getTab("Dumping PID").add("Dumping Velocity", 0).getEntry());
    }

    private void checkLimits() {
		if (linearState != LinearActuatorState.Raised && linearState != LinearActuatorState.TravelingDown
				&& a_linear1.getPosition() >= LINEAR_MAX_TRAVEL - LINEAR_DEADBAND) {
			linearState = LinearActuatorState.Raised;
			linearStop();
		}
		if (linearState != LinearActuatorState.Lowered && linearState != LinearActuatorState.TravelingUp
				&& a_linear1.getPosition() <= LINEAR_MIN_TRAVEL) {
			linearState = LinearActuatorState.Lowered;
			linearStop();
		}
	}

    public class LinearActuatorLimits implements Runnable {

		@Override
		public void run() {
			checkLimits();
		}

	}


}
