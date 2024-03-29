package frc.robot.subsystems.digging;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import frc.robot.custom.LunaMathUtils;
import static frc.robot.Constants.DiggingConstants.*;

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


public class DiggingLeadscrew extends SubsystemBase {
    
    public enum LeadscrewState {
        Extended, Retracted, Traveling, FullExtended, GivenCommand;
    }
    
    private final LunaSparkMax m_leadscrew1;
    private final LunaSparkMax m_leadscrew2;
    private final RelativeEncoder e_leadscrew;
    private final SparkMaxPIDController p_leadscrew;
    private final SparkMaxLimitSwitch l_leadscrew1Top;
    private final SparkMaxLimitSwitch l_leadscrew1Bottom;
    private final SparkMaxLimitSwitch l_leadscrew2Top;
    private final SparkMaxLimitSwitch l_leadscrew2Bottom;

    // NetworkTable Instantiation
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable;

    private boolean leadscrewInitialized = false;
	private LeadscrewState leadscrewState = LeadscrewState.Traveling;
	private LeadscrewStatus leadscrewStatus;

    private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
	private HashMap<String, GenericEntry> pidNTEntries = new HashMap<String, GenericEntry>();
	private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();

    public class LeadscrewStatus {
        public boolean topLimit() 
        {
            return l_leadscrew1Top.isPressed() || l_leadscrew2Top.isPressed();
        }
        public boolean bottomLimit() {
            return l_leadscrew1Bottom.isPressed() || l_leadscrew2Bottom.isPressed();
        }
    }

    public DiggingLeadscrew () {
        m_leadscrew1 = new LunaSparkMax(LEADSCREW_1_CAN_ID, MotorType.kBrushless);
        m_leadscrew2 = new LunaSparkMax(LEADSCREW_2_CAN_ID, MotorType.kBrushless);

        e_leadscrew = m_leadscrew1.getEncoder();
        p_leadscrew = m_leadscrew1.getPIDController();

        // l_leadscrew1Top = m_leadscrew1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        // l_leadscrew1Bottom = m_leadscrew1.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        // l_leadscrew2Top = m_leadscrew2.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        // l_leadscrew2Bottom = m_leadscrew2.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        l_leadscrew1Top = m_leadscrew1.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        l_leadscrew1Bottom = m_leadscrew1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        l_leadscrew2Top = m_leadscrew2.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        l_leadscrew2Bottom = m_leadscrew2.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        leadscrewStatus = new LeadscrewStatus();

        networkTable = ntInstance.getTable("DiggingSubsystem");

        l_leadscrew1Top.enableLimitSwitch(true);
        l_leadscrew1Bottom.enableLimitSwitch(true);
        l_leadscrew2Top.enableLimitSwitch(true);
        l_leadscrew2Bottom.enableLimitSwitch(true);

        createDashboardData();

        if (ENABLE_TEST_DASHBOARDS) {
			reportInitialPID();
			pidConstants.put("leadscrew-kP", LEADSCREW_kP);
			pidConstants.put("leadscrew-kI", LEADSCREW_kI);
			pidConstants.put("leadscrew-kD", LEADSCREW_kD);
			pidConstants.put("leadscrew-kIZ", LEADSCREW_kIZ);
			pidConstants.put("leadscrew-kFF", LEADSCREW_kFF);
			pidConstants.put("leadscrew-setpoint", 0.0);
		}

    }

    @Override
	public void periodic() {
		// This method will be called once per scheduler run
		checkLimits();
		reportSensors();
		if (ENABLE_TEST_DASHBOARDS)
			checkPIDGains();
	}

    private void createDashboardData() {
        shuffleboardEntries.put("screw-init", Shuffleboard.getTab("Competition").add("Screw Init", leadscrewInitialized).withSize(1, 1).withPosition(5, 0).getEntry());
		shuffleboardEntries.put("screw-state", Shuffleboard.getTab("Competition").add("Screw State", leadscrewState.toString()).withSize(1, 1).withPosition(5, 1).getEntry());
		shuffleboardEntries.put("limit-top", Shuffleboard.getTab("Competition").add("Screw Top SW", l_leadscrew1Top.isPressed()).withSize(1, 1).withPosition(5, 2).getEntry());
		shuffleboardEntries.put("screw-position", Shuffleboard.getTab("Competition").add("Screw Position", 0).withSize(1, 1).withPosition(5, 3).getEntry());
		shuffleboardEntries.put("screw-speed", Shuffleboard.getTab("Competition").add("Screw Speed", 0).withSize(1, 1).withPosition(5, 4).getEntry());
		shuffleboardEntries.put("screw-command", Shuffleboard.getTab("Competition").add("Screw Command", 0).withSize(1, 1).withPosition(6, 3).getEntry());
		shuffleboardEntries.put("position-command",
				Shuffleboard.getTab("Competition").getLayout("LSCommand", BuiltInLayouts.kList).withSize(2, 1)
						.withPosition(5, 5).add("Position", 0).getEntry());
    }

    public void leadscrewClearFollower() {
        m_leadscrew2.follow(ExternalFollower.kFollowerDisabled, 0);
    }

    public void leadscrewSetRawSpeed(RobotSide side, double speed) {
        if (side == RobotSide.Left) m_leadscrew1.set(-speed); // right side
        else if (side == RobotSide.Right) m_leadscrew2.set(-speed); // left side
    }

    public boolean leadscrewGetRawLimitSwitch(RobotSide side, MechanismPosition pos) {
        if (side == RobotSide.Left && pos == MechanismPosition.Top) 
            return l_leadscrew1Top.isPressed();
        else if (side == RobotSide.Left && pos == MechanismPosition.Bottom)
            return l_leadscrew1Bottom.isPressed();
        else if (side == RobotSide.Right && pos == MechanismPosition.Top)
            return l_leadscrew2Top.isPressed();
        else if (side == RobotSide.Right && pos == MechanismPosition.Bottom)
            return l_leadscrew2Bottom.isPressed();
        else 
            return false;
    }

    public void leadscrewFinalizeInit() {
        System.out
            .println("Leadscrew Init: Current position is " + e_leadscrew.getPosition() + ". Position reset to 0");
        e_leadscrew.setPosition(0); // Encoder reset
        m_leadscrew2.follow(m_leadscrew1);
        m_leadscrew1.burnFlash();
        m_leadscrew2.burnFlash();
        leadscrewInitialized = true; 
    }

    public boolean isLeadscrewInitialized() {
        return leadscrewInitialized;
    }

    public void checkLimits() {
        if (leadscrewState != LeadscrewState.FullExtended
				&& e_leadscrew.getPosition() >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR) {
			leadscrewState = LeadscrewState.FullExtended;
		}
		if (leadscrewState != LeadscrewState.Retracted && (e_leadscrew.getPosition() < LEADSCREW_MAX_ERROR || l_leadscrew1Top.isPressed() || l_leadscrew2Top.isPressed())) {
			leadscrewState = LeadscrewState.Retracted;
		}
		if (leadscrewState != LeadscrewState.Extended && e_leadscrew.getPosition() > LEADSCREW_MAX_ERROR
				&& Math.abs(e_leadscrew.getPosition() - LEADSCREW_MAX_TRAVEL) >= LEADSCREW_MAX_ERROR) {
			leadscrewState = LeadscrewState.Extended;
		}
    }

    public void leadscrew(LeadscrewState state) {
        if (!leadscrewInitialized) 
            return;
        
        leadscrewState = LeadscrewState.Traveling;
        double command; 
        if (state == LeadscrewState.Extended) command = LEADSCREW_MAX_SPEED*0.5;
        else command = -LEADSCREW_MAX_SPEED;
        m_leadscrew1.set(command);
        shuffleboardEntries.get("screw-command").setDouble(command);
    }

    public void leadscrew(double setpoint) {
        if (!leadscrewInitialized) return;
        leadscrewState = LeadscrewState.Traveling;
        double command = MathUtil.clamp(setpoint, 0, LEADSCREW_MAX_TRAVEL);
        p_leadscrew.setReference(command, ControlType.kSmartMotion);
        shuffleboardEntries.get("screw-command").setDouble(command);
		networkTable.getEntry("leadscrewSetpoint").setDouble(setpoint);
    }

    public void stopLeadScrew() {
        m_leadscrew1.set(0);
    }

    public void leadscrewSpeed(double speed) {
        if (e_leadscrew.getPosition() <= LEADSCREW_MAX_ERROR && speed < 0){
            System.out.println("error1: " + e_leadscrew.getPosition());
            return;
        }
        if (e_leadscrew.getPosition() >= LEADSCREW_MAX_TRAVEL-LEADSCREW_MAX_ERROR && speed > 0){
            System.out.println("error2: " + e_leadscrew.getPosition());
            return;
        }
        leadscrewState = LeadscrewState.Traveling;
        m_leadscrew1.set(speed);
    }

    public LeadscrewState leadscrewState() {
        return leadscrewState;
    }

    public double getDashboardCommandedPosition() {
        return MathUtil.clamp(shuffleboardEntries.get("position-command").getDouble(0), 0, LEADSCREW_MAX_TRAVEL);
    }

    public void commandLeadscrew(double setpoint) {
        leadscrew(setpoint);
        leadscrewState = LeadscrewState.GivenCommand;
    }

    public LeadscrewStatus gLeadscrewStatus() {
        return leadscrewStatus;
    }

    public void reportInitialPID() {
        pidNTEntries.put("leadscrew-kP",
				Shuffleboard.getTab("Digging PID").add("Leadscrew P Gain", LEADSCREW_kP).getEntry());
		pidNTEntries.put("leadscrew-kI",
				Shuffleboard.getTab("Digging PID").add("Leadscrew I Gain", LEADSCREW_kI).getEntry());
		pidNTEntries.put("leadscrew-kD",
				Shuffleboard.getTab("Digging PID").add("Leadscrew D Gain", LEADSCREW_kD).getEntry());
		pidNTEntries.put("leadscrew-kIZ",
				Shuffleboard.getTab("Digging PID").add("Leadscrew I Zone", LEADSCREW_kIZ).getEntry());
		pidNTEntries.put("leadscrew-kFF",
				Shuffleboard.getTab("Digging PID").add("Leadscrew FF", LEADSCREW_kFF).getEntry());
		pidNTEntries.put("leadscrew-setpoint",
				Shuffleboard.getTab("Digging PID").add("Leadscrew Setpoint", 0).getEntry());
		pidNTEntries.put("leadscrew-position",
				Shuffleboard.getTab("Digging PID").add("Leadscrew Position", 0).getEntry());
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
					p_leadscrew.setP(newVal);
					break;

				case "kI":
					p_leadscrew.setI(newVal);
					break;

				case "kD":
					p_leadscrew.setD(newVal);
					break;

				case "kIZ":
					p_leadscrew.setIZone(newVal);
					break;

				case "kFF":
					p_leadscrew.setFF(newVal);
					break;

				case "setpoint":
					p_leadscrew.setReference(newVal, ControlType.kSmartMotion);
					break;

				default:
					break;
			}
			pidConstants.put(pidConstant, newVal);
		}
		pidNTEntries.get("leadscrew-position").setDouble(e_leadscrew.getPosition());
	}

    private void reportSensors() {
        // Initialization Status
		shuffleboardEntries.get("screw-init").setBoolean(leadscrewInitialized);

		// Speed and Position
		shuffleboardEntries.get("screw-state").setString(leadscrewState.toString());
		shuffleboardEntries.get("screw-position").setDouble(LunaMathUtils.roundToPlace(e_leadscrew.getPosition(), 3));
		shuffleboardEntries.get("screw-speed").setDouble(LunaMathUtils.roundToPlace(e_leadscrew.getVelocity(), 3));

		// Limit Switches
		shuffleboardEntries.get("limit-top").setBoolean(l_leadscrew1Top.isPressed());
		//SmartDashboard.putBoolean("Screw Bottom SW", l_leadscrew1Bottom.get());

		// Backend NetworkTable
		networkTable.getEntry("leadscrewInitialized").setBoolean(leadscrewInitialized);
		networkTable.getEntry("leadscrewPosition").setDouble(e_leadscrew.getPosition());
		networkTable.getEntry("leadscrewVelocity").setDouble(e_leadscrew.getVelocity());
		networkTable.getEntry("leadscrewState").setString(leadscrewState.toString());
		networkTable.getEntry("leadscrew1TopLimit").setBoolean(l_leadscrew1Top.isPressed());
		networkTable.getEntry("leadscrew1BottomLimit").setBoolean(l_leadscrew1Bottom.isPressed());
		networkTable.getEntry("leadscrew2TopLimit").setBoolean(l_leadscrew2Top.isPressed());
		networkTable.getEntry("leadscrew2BottomLimit").setBoolean(l_leadscrew2Bottom.isPressed());
		networkTable.getEntry("leadscrew1Current").setDouble(m_leadscrew1.getOutputCurrent());
		networkTable.getEntry("leadscrew2Current").setDouble(m_leadscrew2.getOutputCurrent());
    }




}