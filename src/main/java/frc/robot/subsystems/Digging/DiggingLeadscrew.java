package frc.robot.subsystems.Digging;


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
    private final RelativeEncoder e_leadscrew2;
    private final SparkMaxPIDController p_leadscrew;
    private final SparkMaxPIDController p_leadscrew2;
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
        e_leadscrew2 = m_leadscrew2.getEncoder();
        p_leadscrew = m_leadscrew1.getPIDController();
        p_leadscrew2 = m_leadscrew2.getPIDController();


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
        shuffleboardEntries.put("screw-speed1", Shuffleboard.getTab("Digging PID").add("Screw Speed1", 0).withSize(1, 1).withPosition(0, 4).getEntry());
        shuffleboardEntries.put("screw-speed2", Shuffleboard.getTab("Digging PID").add("Screw Speed2", 0).withSize(1, 1).withPosition(1, 4).getEntry());
        shuffleboardEntries.put("screw-command", Shuffleboard.getTab("Competition").add("Screw Command", 0).withSize(1, 1).withPosition(6, 3).getEntry());
        shuffleboardEntries.put("position-command",
                Shuffleboard.getTab("Competition").getLayout("LSCommand", BuiltInLayouts.kList).withSize(2, 1)
                        .withPosition(5, 5).add("Position", 0).getEntry());
        shuffleboardEntries.put("LeadScrewLeftVoltage", Shuffleboard.getTab("Digging PID").add("LeadScrewLeftVoltage", 0).withSize(1, 1).withPosition(3, 2).getEntry());
        shuffleboardEntries.put("LeadScrewRightVoltage", Shuffleboard.getTab("Digging PID").add("LeadScrewRightVoltage", 0).withSize(1, 1).withPosition(4, 2).getEntry());
                        
                    }


    public void leadscrewClearFollower() {
        // m_leadscrew2.follow(ExternalFollower.kFollowerDisabled, 0);
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
            .println("Leadscrew Init: Current position is " + Math.abs(e_leadscrew.getPosition()) + ". Position reset to 0");
        e_leadscrew.setPosition(0);  // Encoder reset
        e_leadscrew2.setPosition(0); // Encoder reset
        m_leadscrew1.burnFlash();
        m_leadscrew2.burnFlash();
        leadscrewInitialized = true;
    }


    public boolean isLeadscrewInitialized() {
        return leadscrewInitialized;
    }


    public void checkLimits()
    {
        System.out.println(Math.abs(e_leadscrew.getPosition()));
        System.out.println(Math.abs(e_leadscrew2.getPosition()));
        if (leadscrewState != LeadscrewState.FullExtended
                && Math.abs(e_leadscrew.getPosition()) >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR) {
                    leadscrewState = LeadscrewState.FullExtended;
        }
        if (leadscrewState != LeadscrewState.Retracted && (Math.abs(e_leadscrew.getPosition()) < LEADSCREW_MAX_ERROR || l_leadscrew1Top.isPressed() || l_leadscrew2Top.isPressed())) {
            leadscrewState = LeadscrewState.Retracted;
            if(l_leadscrew1Top.isPressed())
            {
                e_leadscrew.setPosition(0);
            }
            if(l_leadscrew2Top.isPressed())
            {
                e_leadscrew2.setPosition(0);
            }
        }
        if (leadscrewState != LeadscrewState.Extended && Math.abs(e_leadscrew.getPosition()) > LEADSCREW_MAX_ERROR
                && Math.abs(Math.abs(e_leadscrew.getPosition()) - LEADSCREW_MAX_TRAVEL) >= LEADSCREW_MAX_ERROR) {
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
        m_leadscrew2.set(command);
        shuffleboardEntries.get("screw-command").setDouble(command);
    }


    public void leadscrew(double setpoint) {
        if (!leadscrewInitialized) return;
        leadscrewState = LeadscrewState.Traveling;
        double command = MathUtil.clamp(setpoint, 0, LEADSCREW_MAX_TRAVEL);
        p_leadscrew.setReference(command, ControlType.kSmartMotion);
        p_leadscrew2.setReference(command, ControlType.kSmartMotion);
        shuffleboardEntries.get("screw-command").setDouble(command);
        networkTable.getEntry("leadscrewSetpoint").setDouble(setpoint);
    }


    public void stopLeadScrew() {
        m_leadscrew1.set(0);
        m_leadscrew2.set(0);
    }


    public void leadscrewSpeed(double speed) {
        if (Math.abs(e_leadscrew.getPosition()) <= LEADSCREW_MAX_ERROR && speed < 0){
            System.out.println("error1: " + Math.abs(e_leadscrew.getPosition()));
            return;
        }
        if (Math.abs(e_leadscrew.getPosition()) >= LEADSCREW_MAX_TRAVEL-LEADSCREW_MAX_ERROR && speed > 0){
            System.out.println("error2: " + Math.abs(e_leadscrew.getPosition()));
            return;
        }
        leadscrewState = LeadscrewState.Traveling;
        m_leadscrew1.set(speed);
        m_leadscrew2.set(speed);
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
                    // p_leadscrew.setP(newVal);
                    // p_leadscrew2.setP(newVal);
                    break;


                case "kI":
                    // p_leadscrew.setI(newVal);
                    // p_leadscrew2.setI(newVal);
                    break;


                case "kD":
                    // p_leadscrew.setD(newVal);
                    // p_leadscrew2.setD(newVal);
                    break;


                case "kIZ":
                    // p_leadscrew.setIZone(newVal);
                    // p_leadscrew2.setIZone(newVal);
                    break;


                case "kFF":
                    // p_leadscrew.setFF(newVal);
                    // p_leadscrew2.setFF(newVal);
                    break;


                case "setpoint":
                    // p_leadscrew.setReference(newVal, ControlType.kSmartMotion);
                    // p_leadscrew2.setReference(newVal, ControlType.kSmartMotion);
                    break;


                default:
                    break;
            }
            pidConstants.put(pidConstant, newVal);
        }
        pidNTEntries.get("leadscrew-position").setDouble(Math.abs(e_leadscrew.getPosition()));
    }


    private void reportSensors() {
        // Initialization Status
        shuffleboardEntries.get("screw-init").setBoolean(leadscrewInitialized);


        // Speed and Position
        shuffleboardEntries.get("screw-state").setString(leadscrewState.toString());
        shuffleboardEntries.get("screw-position").setDouble(LunaMathUtils.roundToPlace(Math.abs(e_leadscrew.getPosition()), 3));
        shuffleboardEntries.get("screw-speed1").setDouble(LunaMathUtils.roundToPlace(e_leadscrew.getVelocity(), 3));
        shuffleboardEntries.get("screw-speed2").setDouble(LunaMathUtils.roundToPlace(e_leadscrew2.getVelocity(), 3));
        shuffleboardEntries.get("LeadScrewLeftVoltage").setDouble(LunaMathUtils.roundToPlace(m_leadscrew1.getBusVoltage(), 3));
        shuffleboardEntries.get("LeadScrewRightVoltage").setDouble(LunaMathUtils.roundToPlace(m_leadscrew2.getBusVoltage(), 3));
        

        // Limit Switches
        shuffleboardEntries.get("limit-top").setBoolean(l_leadscrew1Top.isPressed());
        //SmartDashboard.putBoolean("Screw Bottom SW", l_leadscrew1Bottom.get());


        // Backend NetworkTable
        networkTable.getEntry("leadscrewInitialized").setBoolean(leadscrewInitialized);
        networkTable.getEntry("leadscrewPosition").setDouble(Math.abs(e_leadscrew.getPosition()));
        networkTable.getEntry("leadscrewVelocity").setDouble(e_leadscrew.getVelocity());
        networkTable.getEntry("leadscrewStateRight").setString(leadscrewState.toString());
        networkTable.getEntry("leadscrew1TopLimit").setBoolean(l_leadscrew1Top.isPressed());
        networkTable.getEntry("leadscrew1BottomLimit").setBoolean(l_leadscrew1Bottom.isPressed());
        networkTable.getEntry("leadscrew2TopLimit").setBoolean(l_leadscrew2Top.isPressed());
        networkTable.getEntry("leadscrew2BottomLimit").setBoolean(l_leadscrew2Bottom.isPressed());
        networkTable.getEntry("leadscrew1Current").setDouble(m_leadscrew1.getOutputCurrent());
        networkTable.getEntry("leadscrew2Current").setDouble(m_leadscrew2.getOutputCurrent());
    }

}
//Second Version
// package frc.robot.subsystems.Digging;


// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxLimitSwitch;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMax.ExternalFollower;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import frc.robot.custom.LunaMathUtils;
// import static frc.robot.Constants.DiggingConstants.*;


// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.GlobalConstants.MechanismPosition;
// import frc.robot.Constants.GlobalConstants.RobotSide;
// import frc.robot.custom.LunaSparkMax;
// import static frc.robot.Constants.GlobalConstants.*;


// import java.util.HashMap;

// public class DiggingLeadscrew extends SubsystemBase {
   
//     public enum LeadscrewState {
//         Extended, Retracted, Traveling, FullExtended, GivenCommand;
//     }
   
//     private final LunaSparkMax m_leadscrew1;
//     private final LunaSparkMax m_leadscrew2;
//     private final RelativeEncoder e_leadscrew;
//     private final RelativeEncoder e_leadscrew2;
//     private final SparkMaxPIDController p_leadscrew;
//     private final SparkMaxLimitSwitch l_leadscrew1Top;
//     private final SparkMaxLimitSwitch l_leadscrew1Bottom;
//     private final SparkMaxLimitSwitch l_leadscrew2Top;
//     private final SparkMaxLimitSwitch l_leadscrew2Bottom;


//     // NetworkTable Instantiation
//     private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
//     private final NetworkTable networkTable;


//     private boolean leadscrewInitialized = false;
//     private LeadscrewState leadscrewState = LeadscrewState.Traveling;
//     private LeadscrewStatus leadscrewStatus;


//     private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
//     private HashMap<String, GenericEntry> pidNTEntries = new HashMap<String, GenericEntry>();
//     private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();


//     public class LeadscrewStatus {
//         public boolean topLimit()
//         {
//             return l_leadscrew1Top.isPressed() || l_leadscrew2Top.isPressed();
//         }
//         public boolean bottomLimit() {
//             return l_leadscrew1Bottom.isPressed() || l_leadscrew2Bottom.isPressed();
//         }
//     }


//     public DiggingLeadscrew () {
//         m_leadscrew1 = new LunaSparkMax(LEADSCREW_1_CAN_ID, MotorType.kBrushless);
//         m_leadscrew2 = new LunaSparkMax(LEADSCREW_2_CAN_ID, MotorType.kBrushless);


//         e_leadscrew = m_leadscrew1.getEncoder();
//         e_leadscrew2 = m_leadscrew2.getEncoder();
//         p_leadscrew = m_leadscrew1.getPIDController();


//         l_leadscrew1Top = m_leadscrew1.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
//         l_leadscrew1Bottom = m_leadscrew1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//         l_leadscrew2Top = m_leadscrew2.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
//         l_leadscrew2Bottom = m_leadscrew2.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//         leadscrewStatus = new LeadscrewStatus();


//         networkTable = ntInstance.getTable("DiggingSubsystem");


//         l_leadscrew1Top.enableLimitSwitch(true);
//         l_leadscrew1Bottom.enableLimitSwitch(true);
//         l_leadscrew2Top.enableLimitSwitch(true);
//         l_leadscrew2Bottom.enableLimitSwitch(true);


//         createDashboardData();


//         if (ENABLE_TEST_DASHBOARDS) {
//             reportInitialPID();
//             pidConstants.put("leadscrew-kP", LEADSCREW_kP);
//             pidConstants.put("leadscrew-kI", LEADSCREW_kI);
//             pidConstants.put("leadscrew-kD", LEADSCREW_kD);
//             pidConstants.put("leadscrew-kIZ", LEADSCREW_kIZ);
//             pidConstants.put("leadscrew-kFF", LEADSCREW_kFF);
//             pidConstants.put("leadscrew-setpoint", 0.0);
//         }


//     }


//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//         checkLimits();
//         reportSensors();
//         if (ENABLE_TEST_DASHBOARDS)
//             checkPIDGains();
//     }


//     private void createDashboardData() {
//         shuffleboardEntries.put("screw-init", Shuffleboard.getTab("Competition").add("Screw Init", leadscrewInitialized).withSize(1, 1).withPosition(5, 0).getEntry());
//         shuffleboardEntries.put("screw-state", Shuffleboard.getTab("Competition").add("Screw State", leadscrewState.toString()).withSize(1, 1).withPosition(5, 1).getEntry());
//         shuffleboardEntries.put("limit-top", Shuffleboard.getTab("Competition").add("Screw Top SW", l_leadscrew1Top.isPressed()).withSize(1, 1).withPosition(5, 2).getEntry());
//         shuffleboardEntries.put("screw-position", Shuffleboard.getTab("Competition").add("Screw Position", 0).withSize(1, 1).withPosition(5, 3).getEntry());
//         shuffleboardEntries.put("screw-speed1", Shuffleboard.getTab("Digging PID").add("Screw Speed1", 0).withSize(1, 1).withPosition(0, 4).getEntry());
//         shuffleboardEntries.put("screw-speed2", Shuffleboard.getTab("Digging PID").add("Screw Speed2", 0).withSize(1, 1).withPosition(1, 4).getEntry());
//         shuffleboardEntries.put("screw-command", Shuffleboard.getTab("Competition").add("Screw Command", 0).withSize(1, 1).withPosition(6, 3).getEntry());
//         shuffleboardEntries.put("position-command",
//                 Shuffleboard.getTab("Competition").getLayout("LSCommand", BuiltInLayouts.kList).withSize(2, 1)
//                         .withPosition(5, 5).add("Position", 0).getEntry());
//         shuffleboardEntries.put("LeadScrewLeftVoltage", Shuffleboard.getTab("Digging PID").add("LeadScrewLeftVoltage", 0).withSize(1, 1).withPosition(3, 2).getEntry());
//         shuffleboardEntries.put("LeadScrewRightVoltage", Shuffleboard.getTab("Digging PID").add("LeadScrewRightVoltage", 0).withSize(1, 1).withPosition(4, 2).getEntry());

//     }


//     public void leadscrewClearFollower() {
//         // m_leadscrew2.follow(ExternalFollower.kFollowerDisabled, 0);
//     }


//     public void leadscrewSetRawSpeed(RobotSide side, double speed) {
//         if (side == RobotSide.Left) m_leadscrew1.set(-speed); // right side
//         else if (side == RobotSide.Right) m_leadscrew2.set(-speed); // left side
//     }


//     public boolean leadscrewGetRawLimitSwitch(RobotSide side, MechanismPosition pos) {
//         if (side == RobotSide.Left && pos == MechanismPosition.Top)
//             return l_leadscrew1Top.isPressed();
//         else if (side == RobotSide.Left && pos == MechanismPosition.Bottom)
//             return l_leadscrew1Bottom.isPressed();
//         else if (side == RobotSide.Right && pos == MechanismPosition.Top)
//             return l_leadscrew2Top.isPressed();
//         else if (side == RobotSide.Right && pos == MechanismPosition.Bottom)
//             return l_leadscrew2Bottom.isPressed();
//         else
//             return false;
//     }


//     public void leadscrewFinalizeInit() {
//         System.out
//             .println("Leadscrew Init: Current position is " + Math.abs(e_leadscrew.getPosition()) + ". Position reset to 0");
//         e_leadscrew.setPosition(0);  // Encoder reset
//         e_leadscrew2.setPosition(0); // Encoder reset
//         // m_leadscrew2.follow(m_leadscrew1);
//         m_leadscrew1.burnFlash();
//         m_leadscrew2.burnFlash();
//         leadscrewInitialized = true;
//     }


//     public boolean isLeadscrewInitialized() {
//         return leadscrewInitialized;
//     }


//     public void checkLimits()
//     {
//         System.out.println(Math.abs(e_leadscrew.getPosition()));
//         System.out.println(Math.abs(e_leadscrew2.getPosition()));
//         if (leadscrewState != LeadscrewState.FullExtended
//                 && Math.abs(e_leadscrew.getPosition()) >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR) {
//             leadscrewState = LeadscrewState.FullExtended;
//         }
//         if (leadscrewState != LeadscrewState.Retracted && (Math.abs(e_leadscrew.getPosition()) < LEADSCREW_MAX_ERROR || l_leadscrew1Top.isPressed() || l_leadscrew2Top.isPressed())) {
//             leadscrewState = LeadscrewState.Retracted;
//         }
//         if (leadscrewState != LeadscrewState.Extended && Math.abs(e_leadscrew.getPosition()) > LEADSCREW_MAX_ERROR
//                 && Math.abs(Math.abs(e_leadscrew.getPosition()) - LEADSCREW_MAX_TRAVEL) >= LEADSCREW_MAX_ERROR) {
//             leadscrewState = LeadscrewState.Extended;
//         }
//     }


//     public void leadscrew(LeadscrewState state) {
//         if (!leadscrewInitialized)
//             return;
       
//         leadscrewState = LeadscrewState.Traveling;
//         double command;
//         if (state == LeadscrewState.Extended) command = LEADSCREW_MAX_SPEED*0.5;
//         else command = -LEADSCREW_MAX_SPEED;
//         m_leadscrew1.set(command);
//         m_leadscrew2.set(command);
//         shuffleboardEntries.get("screw-command").setDouble(command);
//     }


//     public void leadscrew(double setpoint) {
//         if (!leadscrewInitialized) return;
//         leadscrewState = LeadscrewState.Traveling;
//         double command = MathUtil.clamp(setpoint, 0, LEADSCREW_MAX_TRAVEL);
//         p_leadscrew.setReference(command, ControlType.kSmartMotion);
//         shuffleboardEntries.get("screw-command").setDouble(command);
//         networkTable.getEntry("leadscrewSetpoint").setDouble(setpoint);
//     }


//     public void stopLeadScrew() {
//         m_leadscrew1.set(0);
//         m_leadscrew2.set(0);

        
//     }


//     public void leadscrewSpeed(double speed) {
//         if (Math.abs(e_leadscrew.getPosition()) <= LEADSCREW_MAX_ERROR && speed < 0){
//             System.out.println("error1: " + Math.abs(e_leadscrew.getPosition()));
//             return;
//         }
//         if (Math.abs(e_leadscrew.getPosition()) >= LEADSCREW_MAX_TRAVEL-LEADSCREW_MAX_ERROR && speed > 0){
//             System.out.println("error2: " + Math.abs(e_leadscrew.getPosition()));
//             return;
//         }
//         leadscrewState = LeadscrewState.Traveling;
//         m_leadscrew1.set(speed);
//         m_leadscrew2.set(speed);
//     }


//     public LeadscrewState leadscrewState() {
//         return leadscrewState;
//     }


//     public double getDashboardCommandedPosition() {
//         return MathUtil.clamp(shuffleboardEntries.get("position-command").getDouble(0), 0, LEADSCREW_MAX_TRAVEL);
//     }


//     public void commandLeadscrew(double setpoint) {
//         leadscrew(setpoint);
//         leadscrewState = LeadscrewState.GivenCommand;
//     }


//     public LeadscrewStatus gLeadscrewStatus() {
//         return leadscrewStatus;
//     }


//     public void reportInitialPID() {
//         pidNTEntries.put("leadscrew-kP",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew P Gain", LEADSCREW_kP).getEntry());
//         pidNTEntries.put("leadscrew-kI",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew I Gain", LEADSCREW_kI).getEntry());
//         pidNTEntries.put("leadscrew-kD",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew D Gain", LEADSCREW_kD).getEntry());
//         pidNTEntries.put("leadscrew-kIZ",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew I Zone", LEADSCREW_kIZ).getEntry());
//         pidNTEntries.put("leadscrew-kFF",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew FF", LEADSCREW_kFF).getEntry());
//         pidNTEntries.put("leadscrew-setpoint",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew Setpoint", 0).getEntry());
//         pidNTEntries.put("leadscrew-position",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew Position", 0).getEntry());
//     }


//     private void checkPIDGains() {
//         for (String pidConstant : pidConstants.keySet()) {
//             double newVal = pidNTEntries.get(pidConstant).getDouble(pidConstants.get(pidConstant));
//             if (newVal == pidConstants.get(pidConstant))
//                 continue;
//             // System.out.println("LS: " + pidConstant + " value updated: " + newVal);
//             String[] split = pidConstant.split("-", 2);
//             switch (split[1]) {
//                 case "kP":
//                     p_leadscrew.setP(newVal);
//                     break;


//                 case "kI":
//                     p_leadscrew.setI(newVal);
//                     break;


//                 case "kD":
//                     p_leadscrew.setD(newVal);
//                     break;


//                 case "kIZ":
//                     p_leadscrew.setIZone(newVal);
//                     break;


//                 case "kFF":
//                     p_leadscrew.setFF(newVal);
//                     break;


//                 case "setpoint":
//                     p_leadscrew.setReference(newVal, ControlType.kSmartMotion);
//                     break;


//                 default:
//                     break;
//             }
//             pidConstants.put(pidConstant, newVal);
//         }
//         pidNTEntries.get("leadscrew-position").setDouble(Math.abs(e_leadscrew.getPosition()));
//     }


//     private void reportSensors() {
//         // Initialization Status
//         shuffleboardEntries.get("screw-init").setBoolean(leadscrewInitialized);


//         // Speed and Position
//         shuffleboardEntries.get("screw-state").setString(leadscrewState.toString());
//         shuffleboardEntries.get("screw-position").setDouble(LunaMathUtils.roundToPlace(Math.abs(e_leadscrew.getPosition()), 3));
//         shuffleboardEntries.get("screw-speed1").setDouble(LunaMathUtils.roundToPlace(e_leadscrew.getVelocity(), 3));
//         shuffleboardEntries.get("screw-speed2").setDouble(LunaMathUtils.roundToPlace(e_leadscrew2.getVelocity(), 3));
//         shuffleboardEntries.get("LeadScrewLeftVoltage").setDouble(LunaMathUtils.roundToPlace(m_leadscrew1.getBusVoltage(), 3));
//         shuffleboardEntries.get("LeadScrewRightVoltage").setDouble(LunaMathUtils.roundToPlace(m_leadscrew2.getBusVoltage(), 3));


//         // Limit Switches
//         shuffleboardEntries.get("limit-top").setBoolean(l_leadscrew1Top.isPressed());
//         //SmartDashboard.putBoolean("Screw Bottom SW", l_leadscrew1Bottom.get());


//         // Backend NetworkTable
//         networkTable.getEntry("leadscrewInitialized").setBoolean(leadscrewInitialized);
//         networkTable.getEntry("leadscrewPosition").setDouble(Math.abs(e_leadscrew.getPosition()));
//         networkTable.getEntry("leadscrewVelocity").setDouble(e_leadscrew.getVelocity());
//         networkTable.getEntry("leadscrewStateRight").setString(leadscrewState.toString());
//         networkTable.getEntry("leadscrew1TopLimit").setBoolean(l_leadscrew1Top.isPressed());
//         networkTable.getEntry("leadscrew1BottomLimit").setBoolean(l_leadscrew1Bottom.isPressed());
//         networkTable.getEntry("leadscrew2TopLimit").setBoolean(l_leadscrew2Top.isPressed());
//         networkTable.getEntry("leadscrew2BottomLimit").setBoolean(l_leadscrew2Bottom.isPressed());
//         networkTable.getEntry("leadscrew1Current").setDouble(m_leadscrew1.getOutputCurrent());
//         networkTable.getEntry("leadscrew2Current").setDouble(m_leadscrew2.getOutputCurrent());
//     }

// }


// package frc.robot.subsystems.Digging;


// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxLimitSwitch;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMax.ExternalFollower;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import frc.robot.custom.LunaMathUtils;
// import static frc.robot.Constants.DiggingConstants.*;


// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.GlobalConstants.MechanismPosition;
// import frc.robot.Constants.GlobalConstants.RobotSide;
// import frc.robot.custom.LunaSparkMax;
// import static frc.robot.Constants.GlobalConstants.*;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// import java.util.HashMap;

// public class DiggingLeadscrew extends SubsystemBase {
   
//     public enum LeadscrewStateRight {
//         Extended, Retracted, Traveling, FullExtended, GivenCommand;
//     }

//     public enum LeadscrewStateLeft {
//         Extended, Retracted, Traveling, FullExtended, GivenCommand;
//     }
   
//     private final LunaSparkMax m_leadscrew1;
//     private final LunaSparkMax m_leadscrew2;
//     private final RelativeEncoder e_leadscrew;
//     private final RelativeEncoder e_leadscrew2;
//     private final SparkMaxPIDController p_leadscrew;
//     private final SparkMaxPIDController p_leadscrew2;
//     private final SparkMaxLimitSwitch l_leadscrew1Top;
//     private final SparkMaxLimitSwitch l_leadscrew1Bottom;
//     private final SparkMaxLimitSwitch l_leadscrew2Top;
//     private final SparkMaxLimitSwitch l_leadscrew2Bottom;



//     // NetworkTable Instantiation
//     private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
//     private final NetworkTable networkTable;


//     private boolean leadscrewInitialized = false;
//     private LeadscrewStateRight leadscrewStateRight = LeadscrewStateRight.Traveling;
//     private LeadscrewStateLeft leadscrewStateLeft = LeadscrewStateLeft.Traveling;

//     private LeadscrewStatusRight leadscrewStatusRight;
//     private LeadscrewStatusLeft leadscrewStatusLeft;



//     private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
//     private HashMap<String, GenericEntry> pidNTEntries = new HashMap<String, GenericEntry>();
//     private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();


//     public class LeadscrewStatusRight 
//     {
//         public boolean topLimit()
//         {
//             return l_leadscrew1Top.isPressed();
//         }
//         public boolean bottomLimit() {
//             return l_leadscrew1Bottom.isPressed();
//         }
//     }

//     public class LeadscrewStatusLeft 
//     {
//         public boolean topLimit()
//         {
//             return l_leadscrew2Top.isPressed();
//         }
//         public boolean bottomLimit() {
//             return l_leadscrew2Bottom.isPressed();
//         }
//     }


//     public DiggingLeadscrew () {
//         m_leadscrew1 = new LunaSparkMax(LEADSCREW_2_CAN_ID, MotorType.kBrushless);
//         m_leadscrew2 = new LunaSparkMax(LEADSCREW_1_CAN_ID, MotorType.kBrushless);


//         e_leadscrew = m_leadscrew1.getEncoder();
//         e_leadscrew2 = m_leadscrew2.getEncoder();
//         p_leadscrew = m_leadscrew1.getPIDController();
//         p_leadscrew2 = m_leadscrew2.getPIDController();


//         l_leadscrew1Top = m_leadscrew1.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
//         l_leadscrew1Bottom = m_leadscrew1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//         l_leadscrew2Top = m_leadscrew2.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
//         l_leadscrew2Bottom = m_leadscrew2.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//         leadscrewStatusRight = new LeadscrewStatusRight();
//         leadscrewStatusLeft = new LeadscrewStatusLeft();



//         networkTable = ntInstance.getTable("DiggingSubsystem");


//         l_leadscrew1Top.enableLimitSwitch(true);
//         l_leadscrew1Bottom.enableLimitSwitch(true);
//         l_leadscrew2Top.enableLimitSwitch(true);
//         l_leadscrew2Bottom.enableLimitSwitch(true);


//         createDashboardData();


//         if (ENABLE_TEST_DASHBOARDS) {
//             reportInitialPID();
//             pidConstants.put("leadscrew-kP", LEADSCREW_kP);
//             pidConstants.put("leadscrew-kI", LEADSCREW_kI);
//             pidConstants.put("leadscrew-kD", LEADSCREW_kD);
//             pidConstants.put("leadscrew-kIZ", LEADSCREW_kIZ);
//             pidConstants.put("leadscrew-kFF", LEADSCREW_kFF);
//             pidConstants.put("leadscrew-setpoint", 0.0);
//         }


//     }


//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//         checkLimits();
//         reportSensors();
//         if (ENABLE_TEST_DASHBOARDS)
//             checkPIDGains();
//     }


//     private void createDashboardData() {
//         shuffleboardEntries.put("screw-init", Shuffleboard.getTab("Competition").add("Screw Init", leadscrewInitialized).withSize(1, 1).withPosition(5, 0).getEntry());
//         shuffleboardEntries.put("screw-state", Shuffleboard.getTab("Competition").add("Screw State", leadscrewStateRight.toString()).withSize(1, 1).withPosition(5, 1).getEntry());
//         shuffleboardEntries.put("limit-top", Shuffleboard.getTab("Competition").add("Screw Top SW", l_leadscrew1Top.isPressed()).withSize(1, 1).withPosition(5, 2).getEntry());
//         shuffleboardEntries.put("screw-position", Shuffleboard.getTab("Competition").add("Screw Position", 0).withSize(1, 1).withPosition(5, 3).getEntry());
//         shuffleboardEntries.put("screw-speed1", Shuffleboard.getTab("Digging PID").add("Screw Speed1", 0).withSize(1, 1).withPosition(0, 4).getEntry());
//         shuffleboardEntries.put("screw-speed2", Shuffleboard.getTab("Digging PID").add("Screw Speed2", 0).withSize(1, 1).withPosition(1, 4).getEntry());
//         shuffleboardEntries.put("screw-command", Shuffleboard.getTab("Competition").add("Screw Command", 0).withSize(1, 1).withPosition(6, 3).getEntry());
//         shuffleboardEntries.put("position-command",
//                 Shuffleboard.getTab("Competition").getLayout("LSCommand", BuiltInLayouts.kList).withSize(2, 1)
//                         .withPosition(5, 5).add("Position", 0).getEntry());

//         shuffleboardEntries.put("LeadScrewLeftVoltage", Shuffleboard.getTab("Digging PID").add("LeadScrewLeftVoltage", 0).withSize(1, 1).withPosition(3, 2).getEntry());
//         shuffleboardEntries.put("LeadScrewRightVoltage", Shuffleboard.getTab("Digging PID").add("LeadScrewRightVoltage", 0).withSize(1, 1).withPosition(4, 2).getEntry());
//     }


//     public void leadscrewClearFollower() {
//         // m_leadscrew2.follow(ExternalFollower.kFollowerDisabled, 0);
//     }


//     public void leadscrewSetRawSpeed(RobotSide side, double speed) {
//         if (side == RobotSide.Left) m_leadscrew1.set(-speed); // right side
//         else if (side == RobotSide.Right) m_leadscrew2.set(-speed); // left side
//     }


//     public boolean leadscrewGetRawLimitSwitch(RobotSide side, MechanismPosition pos) {
//         if (side == RobotSide.Left && pos == MechanismPosition.Top)
//             return l_leadscrew1Top.isPressed();
//         else if (side == RobotSide.Left && pos == MechanismPosition.Bottom)
//             return l_leadscrew1Bottom.isPressed();
//         else if (side == RobotSide.Right && pos == MechanismPosition.Top)
//             return l_leadscrew2Top.isPressed();
//         else if (side == RobotSide.Right && pos == MechanismPosition.Bottom)
//             return l_leadscrew2Bottom.isPressed();
//         else
//             return false;
//     }


//     public void leadscrewFinalizeInit() {
//         System.out
//             .println("Leadscrew Init: Current position is " + Math.abs(e_leadscrew.getPosition()) + ". Position reset to 0");
//         e_leadscrew.setPosition(0);  // Encoder reset
//         e_leadscrew2.setPosition(0); // Encoder reset
//         //m_leadscrew2.follow(m_leadscrew1);
//         m_leadscrew1.burnFlash();
//         m_leadscrew2.burnFlash();
//         leadscrewInitialized = true;
//     }


//     public boolean isLeadscrewInitialized() {
//         return leadscrewInitialized;
//     }


//     public void checkLimits()
//     {
//         System.out.println(Math.abs(e_leadscrew.getPosition()));
//         System.out.println(Math.abs(e_leadscrew2.getPosition()));
//         if (leadscrewStateRight != LeadscrewStateRight.FullExtended
//                 && Math.abs(e_leadscrew.getPosition()) >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR) {
//             leadscrewStateRight = LeadscrewStateRight.FullExtended;
//         }
//         if (leadscrewStateRight != LeadscrewStateRight.Retracted && (Math.abs(e_leadscrew.getPosition()) < LEADSCREW_MAX_ERROR || l_leadscrew1Top.isPressed() || l_leadscrew2Top.isPressed())) {
//             leadscrewStateRight = LeadscrewStateRight.Retracted;
//         }
//         if (leadscrewStateRight != LeadscrewStateRight.Extended && Math.abs(e_leadscrew.getPosition()) > LEADSCREW_MAX_ERROR
//                 && Math.abs(Math.abs(e_leadscrew.getPosition()) - LEADSCREW_MAX_TRAVEL) >= LEADSCREW_MAX_ERROR) {
//             leadscrewStateRight = LeadscrewStateRight.Extended;
//         }
//         if (leadscrewStateLeft != LeadscrewStateLeft.FullExtended
//                 && Math.abs(e_leadscrew.getPosition()) >= LEADSCREW_MAX_TRAVEL - LEADSCREW_MAX_ERROR) {
//             leadscrewStateLeft = LeadscrewStateLeft.FullExtended;
//         }
//         if (leadscrewStateLeft != LeadscrewStateLeft.Retracted && (Math.abs(e_leadscrew.getPosition()) < LEADSCREW_MAX_ERROR || l_leadscrew1Top.isPressed() || l_leadscrew2Top.isPressed())) {
//             leadscrewStateLeft = LeadscrewStateLeft.Retracted;
//         }
//         if (leadscrewStateLeft != LeadscrewStateLeft.Extended && Math.abs(e_leadscrew.getPosition()) > LEADSCREW_MAX_ERROR
//                 && Math.abs(Math.abs(e_leadscrew.getPosition()) - LEADSCREW_MAX_TRAVEL) >= LEADSCREW_MAX_ERROR) {
//                 leadscrewStateLeft = LeadscrewStateLeft.Extended;
//         }
//     }


//     public void leadscrewRight(LeadscrewStateRight state) {
//         if (!leadscrewInitialized)
//             return;
       
//         leadscrewStateRight = LeadscrewStateRight.Traveling;
//         double command;
//         if (state == LeadscrewStateRight.Extended) command = LEADSCREW_MAX_SPEED*0.5;
//         else command = -LEADSCREW_MAX_SPEED;
//         m_leadscrew1.set(command);
//         shuffleboardEntries.get("screw-command").setDouble(command);
//     }
//     public void leadscrewLeft(LeadscrewStateLeft state) {
//         if (!leadscrewInitialized)
//             return;
       
//         leadscrewStateLeft = LeadscrewStateLeft.Traveling;
//         double command;
//         if (state == LeadscrewStateLeft.Extended) command = LEADSCREW_MAX_SPEED*0.5;
//         else command = -LEADSCREW_MAX_SPEED;
//         m_leadscrew2.set(command);
//         shuffleboardEntries.get("screw-command").setDouble(command);
//     }


//     public void leadscrewRight(double setpoint) {
//         if (!leadscrewInitialized) return;
//         leadscrewStateRight = LeadscrewStateRight.Traveling;
//         double command = MathUtil.clamp(setpoint, 0, LEADSCREW_MAX_TRAVEL);
//         p_leadscrew.setReference(command, ControlType.kSmartMotion);
//         shuffleboardEntries.get("screw-command").setDouble(command);
//         networkTable.getEntry("leadscrewSetpoint").setDouble(setpoint);
//     }
//     public void leadscrewLeft(double setpoint) {
//         if (!leadscrewInitialized) return;
//         leadscrewStateLeft = LeadscrewStateLeft.Traveling;
//         double command = MathUtil.clamp(setpoint, 0, LEADSCREW_MAX_TRAVEL);
//         p_leadscrew2.setReference(command, ControlType.kSmartMotion);
//         shuffleboardEntries.get("screw-command").setDouble(command);
//         networkTable.getEntry("leadscrewSetpoint").setDouble(setpoint);
//     }


//     public void stopLeadScrew() {
//         m_leadscrew1.set(0);
//         m_leadscrew2.set(0);
//     }


//     public void leadscrewSpeedRight(double speed) {
//         if (Math.abs(e_leadscrew.getPosition()) <= LEADSCREW_MAX_ERROR && speed < 0){
//             System.out.println("error1: " + Math.abs(e_leadscrew.getPosition()));
//             return;
//         }
//         if (Math.abs(e_leadscrew.getPosition()) >= LEADSCREW_MAX_TRAVEL-LEADSCREW_MAX_ERROR && speed > 0){
//             System.out.println("error2: " + Math.abs(e_leadscrew.getPosition()));
//             return;
//         }
//         leadscrewStateRight = LeadscrewStateRight.Traveling;
//         m_leadscrew1.set(speed);
//     }

//     public void leadscrewSpeedLeft(double speed) {
//         if (Math.abs(e_leadscrew2.getPosition()) <= LEADSCREW_MAX_ERROR && speed < 0){
//             System.out.println("error1: " + Math.abs(e_leadscrew2.getPosition()));
//             return;
//         }
//         if (Math.abs(e_leadscrew2.getPosition()) >= LEADSCREW_MAX_TRAVEL-LEADSCREW_MAX_ERROR && speed > 0){
//             System.out.println("error2: " + Math.abs(e_leadscrew2.getPosition()));
//             return;
//         }
//         leadscrewStateLeft = LeadscrewStateLeft.Traveling;
//         m_leadscrew2.set(speed);
//     }


//     public LeadscrewStateRight leadscrewStateRight() {
//         return leadscrewStateRight;
//     }

//     public LeadscrewStateLeft leadscrewStateLeft() {
//         return leadscrewStateLeft;
//     }


//     public double getDashboardCommandedPosition() {
//         return MathUtil.clamp(shuffleboardEntries.get("position-command").getDouble(0), 0, LEADSCREW_MAX_TRAVEL);
//     }


//     public void commandLeadscrewRight(double setpoint) {
//         leadscrewRight(setpoint);
//         leadscrewStateRight = LeadscrewStateRight.GivenCommand;
//     }

//     public void commandLeadscrewLeft(double setpoint) {
//         leadscrewLeft(setpoint);
//         leadscrewStateLeft = LeadscrewStateLeft.GivenCommand;
//     }


//     public LeadscrewStatusRight gLeadscrewStatusRight() {
//         return leadscrewStatusRight;
//     }

//     public LeadscrewStatusLeft gLeadscrewStatusLeft() {
//         return leadscrewStatusLeft;
//     }


//     public void reportInitialPID() {
//         pidNTEntries.put("leadscrew-kP",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew P Gain", LEADSCREW_kP).getEntry());
//         pidNTEntries.put("leadscrew-kI",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew I Gain", LEADSCREW_kI).getEntry());
//         pidNTEntries.put("leadscrew-kD",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew D Gain", LEADSCREW_kD).getEntry());
//         pidNTEntries.put("leadscrew-kIZ",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew I Zone", LEADSCREW_kIZ).getEntry());
//         pidNTEntries.put("leadscrew-kFF",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew FF", LEADSCREW_kFF).getEntry());
//         pidNTEntries.put("leadscrew-setpoint",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew Setpoint", 0).getEntry());
//         pidNTEntries.put("leadscrew-position",
//                 Shuffleboard.getTab("Digging PID").add("Leadscrew Position", 0).getEntry());
//     }


//     private void checkPIDGains() {
//         for (String pidConstant : pidConstants.keySet()) {
//             double newVal = pidNTEntries.get(pidConstant).getDouble(pidConstants.get(pidConstant));
//             if (newVal == pidConstants.get(pidConstant))
//                 continue;
//             // System.out.println("LS: " + pidConstant + " value updated: " + newVal);
//             String[] split = pidConstant.split("-", 2);
//             switch (split[1]) {
//                 case "kP":
//                     p_leadscrew.setP(newVal);
//                     break;


//                 case "kI":
//                     p_leadscrew.setI(newVal);
//                     break;


//                 case "kD":
//                     p_leadscrew.setD(newVal);
//                     break;


//                 case "kIZ":
//                     p_leadscrew.setIZone(newVal);
//                     break;


//                 case "kFF":
//                     p_leadscrew.setFF(newVal);
//                     break;


//                 case "setpoint":
//                     p_leadscrew.setReference(newVal, ControlType.kSmartMotion);
//                     break;


//                 default:
//                     break;
//             }
//             pidConstants.put(pidConstant, newVal);
//         }
//         pidNTEntries.get("leadscrew-position").setDouble(Math.abs(e_leadscrew.getPosition()));

//         // double linearP = SmartDashboard.getNumber("linearP", 0.0);
//         // double linearI = SmartDashboard.getNumber("linearI", 0.0);
//         // double linearD = SmartDashboard.getNumber("linearD", 0.0);
//         // double linearFF = SmartDashboard.getNumber("linearFF", 0.0);
//         // double setPt = SmartDashboard.getNumber("Target Position", 0.0);

//         // if (linearP != p_leadscrew2.getP()) p_leadscrew2.setP(linearP);
//         // if (linearP != p_leadscrew2.getI()) p_leadscrew2.setI(linearI);
//         // if (linearP != p_leadscrew2.getD()) p_leadscrew2.setD(linearD);
//         // if (linearP != p_leadscrew2.getFF()) p_leadscrew2.setFF(linearFF);
//         // SmartDashboard.putNumber("linearP", p_leadscrew2.getP());
//         // SmartDashboard.putNumber("linearI", p_leadscrew2.getI());
//         // SmartDashboard.putNumber("linearD", p_leadscrew2.getD());
//         // SmartDashboard.putNumber("linearFF", p_leadscrew2.getFF());
//         // SmartDashboard.putNumber("Target Position", setPt);
//     }


//     private void reportSensors() {
//         // Initialization Status
//         shuffleboardEntries.get("screw-init").setBoolean(leadscrewInitialized);


//         // Speed and Position
//         shuffleboardEntries.get("screw-state").setString(leadscrewStateRight.toString());
//         shuffleboardEntries.get("screw-position").setDouble(LunaMathUtils.roundToPlace(Math.abs(e_leadscrew.getPosition()), 3));
//         shuffleboardEntries.get("screw-speed1").setDouble(LunaMathUtils.roundToPlace(e_leadscrew.getVelocity(), 3));
//         shuffleboardEntries.get("screw-speed2").setDouble(LunaMathUtils.roundToPlace(e_leadscrew2.getVelocity(), 3));
//         shuffleboardEntries.get("LeadScrewLeftVoltage").setDouble(LunaMathUtils.roundToPlace(m_leadscrew1.getBusVoltage(), 3));
//         shuffleboardEntries.get("LeadScrewRightVoltage").setDouble(LunaMathUtils.roundToPlace(m_leadscrew2.getBusVoltage(), 3));

//         // Limit Switches
//         shuffleboardEntries.get("limit-top").setBoolean(l_leadscrew1Top.isPressed());
//         //SmartDashboard.putBoolean("Screw Bottom SW", l_leadscrew1Bottom.get());


//         // Backend NetworkTable
//         networkTable.getEntry("leadscrewInitialized").setBoolean(leadscrewInitialized);
//         networkTable.getEntry("leadscrewPosition").setDouble(Math.abs(e_leadscrew.getPosition()));
//         networkTable.getEntry("leadscrewVelocity").setDouble(e_leadscrew.getVelocity());
//         networkTable.getEntry("leadscrewStateRight").setString(leadscrewStateRight.toString());
//         networkTable.getEntry("leadscrew1TopLimit").setBoolean(l_leadscrew1Top.isPressed());
//         networkTable.getEntry("leadscrew1BottomLimit").setBoolean(l_leadscrew1Bottom.isPressed());
//         networkTable.getEntry("leadscrew2TopLimit").setBoolean(l_leadscrew2Top.isPressed());
//         networkTable.getEntry("leadscrew2BottomLimit").setBoolean(l_leadscrew2Bottom.isPressed());
//         networkTable.getEntry("leadscrew1Current").setDouble(m_leadscrew1.getOutputCurrent());
//         networkTable.getEntry("leadscrew2Current").setDouble(m_leadscrew2.getOutputCurrent());
//     }

// }