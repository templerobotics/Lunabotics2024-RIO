import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
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
import frc.robot.Constants.GlobalConstants.MechanismPosition;
import frc.robot.Constants.GlobalConstants.RobotSide;
import frc.robot.custom.LunaSparkMax;
import static frc.robot.Constants.GlobalConstants.*;

import java.util.HashMap;

public class DiggingLinearActuator extends SubsystemBase {
    
    public enum LinearActuatorState {
		Unknown, Raised, Lowered, TravelingUp, TravelingDown, Commanded;
	}

    private final LunaSparkMax m_linear1;
    private final LunaSparkMax m_linear2;

    private final RelativeEncoder e_linear1;
    private final RelativeEncoder e_linear2;
    private final SparkMaxPIDController p_linear;

    // NetworkTable Instantiation
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable;

    private boolean linearInitialized = false;
    private LinearActuatorState linearState = LinearActuatorState.Unknown;
    private Runnable linearRunnable;
    private Notifier linearNotifier;

    //Hash Map for PID constants
    private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
    //Hash Map for Network Table
    private HashMap<String, GenericEntry> pidNTEntries = new HashMap<String, GenericEntry>();
    //Hash Map for Shuffleboard
    private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();

    public DiggingLinearActuator() {
        m_linear1 = new LunaSparkMax(LINEAR_1_CAN_ID, MotorType.kBrushed);
        m_linear2 = new LunaSparkMax(LINEAR_2_CAN_ID, MotorType.kBrushed);

        e_linear1 = m_linear1.getEncoder();
        e_linear2 = m_linear2.getEncoder();
        p_linear = m_linear1.getPIDController();

        e_linear1.setInverted(LINEAR_INVERT);
        e_linear2.setInverted(LINEAR_INVERT);

        m_linear1.burnFlash();
        m_linear2.burnFlash();

        networkTable = ntInstance.getTable("DiggingSubsystem");

        createDashboardData();
        linearStop();
    
        if (ENABLE_TEST_DASHBOARDS){
            reportInitialPID();
            pidConstants.put("linear-kP", LINEAR_kP);
            pidConstants.put("linear-kI", LINEAR_kI);
            pidConstants.put("linear-kD", LINEAR_kD);
            pidConstants.put("linear-kIZ", LINEAR_kIZ);
            pidConstants.put("linear-kFF", LINEAR_kFF);
            pidConstants.put("linear-setpoint", 0.0);
        }
    }

    private void createDashboardData() {
		shuffleboardEntries.put("linear-init", Shuffleboard.getTab("Competition").add("Linear Init", linearInitialized)
				.withSize(1, 1).withPosition(3, 0).getEntry());
		shuffleboardEntries.put("linear-state", Shuffleboard.getTab("Competition")
				.add("Linear State", linearState.toString()).withSize(1, 1).withPosition(3, 1).getEntry());
		shuffleboardEntries.put("position-1",
				Shuffleboard.getTab("Competition").add("L1 Position", 0).withSize(1, 1).withPosition(3, 2).getEntry());
		shuffleboardEntries.put("position-2",
				Shuffleboard.getTab("Competition").add("L2 Position", 0).withSize(1, 1).withPosition(3, 3).getEntry());
		shuffleboardEntries.put("position-command",
				Shuffleboard.getTab("Competition").getLayout("LACommand", BuiltInLayouts.kList).withSize(1, 2)
						.withPosition(3, 4).add("Position", 0.0).getEntry());
	}

    @Override
    public void periodic(){
        //This will be called once per scheduler run
        reportSensors();
        checkPIDGains();
    }

    private void linearUp() {
        if (e_linear1.getPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)
				|| e_linear2.getPosition() >= (LINEAR_MAX_TRAVEL - LINEAR_DEADBAND)
				|| linearState == LinearActuatorState.Raised)
			return;
        linearState = LinearActuatorState.TravelingUp;
        m_linear1.set(1);
        m_linear2.set(-1);
    }

    private void linearDown() {
		if (e_linear1.getPosition() <= LINEAR_MIN_TRAVEL || e_linear2.getPosition() <= LINEAR_MIN_TRAVEL
				|| linearState == LinearActuatorState.Lowered)
			return;
		linearState = LinearActuatorState.TravelingDown;
        m_linear1.set(-1);
        m_linear2.set(1);
    }

    private void linearStop() {
		m_linear1.set(0);
		m_linear2.set(0);
	}

	public void linearActuatorInitStart() {
		linearDown();
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

    public double getLinearPosition(RobotSide side) {
        if (side == RobotSide.Left) return e_linear1.getPosition();
        else return e_linear2.getPosition() + LINEAR_2_ADJUSTMENT;
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
					p_linear.setReference(newVal, ControlType.kSmartMotion);
					break;

				default:
					break;
			}
			pidConstants.put(pidConstant, newVal);
		}
		pidNTEntries.get("p_linear1-position").setDouble(e_linear1.getPosition());
        pidNTEntries.get("p_linear2-position").setDouble(e_linear2.getPosition());
	}

    private void reportSensors() {
		shuffleboardEntries.get("linear-init").setBoolean(linearInitialized);
		shuffleboardEntries.get("linear-state").setString(linearState.toString());
        shuffleboardEntries.get("position-1").setDouble(LunaMathUtils.roundToPlace(e_linear1.getPosition(), 3));
        shuffleboardEntries.get("position-2").setDouble(LunaMathUtils.roundToPlace(e_linear2.getPosition(), 3));
		networkTable.getEntry("linearInitialized").setBoolean(linearInitialized);
        networkTable.getEntry("linear1Position").setDouble(e_linear1.getPosition());
        networkTable.getEntry("linear2Position").setDouble(e_linear2.getPosition());
		networkTable.getEntry("linearState").setString(linearState.toString());
	}

    
    private void reportInitialPID(){
        pidNTEntries.put("belt-kP", Shuffleboard.getTab("Digging PID").add("Belt P Gain", BELT_kP).getEntry());
		pidNTEntries.put("belt-kI", Shuffleboard.getTab("Digging PID").add("Belt I Gain", BELT_kI).getEntry());
		pidNTEntries.put("belt-kD", Shuffleboard.getTab("Digging PID").add("Belt D Gain", BELT_kD).getEntry());
		pidNTEntries.put("belt-kIZ", Shuffleboard.getTab("Digging PID").add("Belt I Zone", BELT_kIZ).getEntry());
		pidNTEntries.put("belt-kFF", Shuffleboard.getTab("Digging PID").add("Belt FF", BELT_kFF).getEntry());
		pidNTEntries.put("belt-setpoint", Shuffleboard.getTab("Digging PID").add("Belt Setpoint", 0).getEntry());
		pidNTEntries.put("belt-velocity", Shuffleboard.getTab("Digging PID").add("Belt Velocity", 0).getEntry());
    }

    private void checkLimits() {
		if (linearState != LinearActuatorState.Raised && linearState != LinearActuatorState.TravelingDown
				&& e_linear1.getPosition() >= LINEAR_MAX_TRAVEL - LINEAR_DEADBAND
				&& e_linear2.getPosition() >= LINEAR_MAX_TRAVEL - LINEAR_DEADBAND) {
			linearState = LinearActuatorState.Raised;
			linearStop();
		}
		if (linearState != LinearActuatorState.Lowered && linearState != LinearActuatorState.TravelingUp
				&& e_linear1.getPosition() <= LINEAR_MIN_TRAVEL && e_linear2.getPosition() <= LINEAR_MIN_TRAVEL) {
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
