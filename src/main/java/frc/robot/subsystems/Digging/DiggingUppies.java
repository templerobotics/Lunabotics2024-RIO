package frc.robot.subsystems.Digging;


import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;


import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxRelativeEncoder;


import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import frc.robot.custom.LunaMathUtils;
import static frc.robot.Constants.DiggingConstants.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;


import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.custom.LunaSparkMax;
import static frc.robot.Constants.GlobalConstants.*;


import java.util.HashMap;


public class DiggingUppies extends SubsystemBase {
    public enum LinearActuatorStateRight {
        Unknown, Raised, Lowered, TravelingUp, TravelingDown, Commanded;
    }
    public enum LinearActuatorStateLeft {
        Unknown, Raised, Lowered, TravelingUp, TravelingDown, Commanded;
    }


    private final LunaSparkMax m_linear1;
    private final LunaSparkMax m_linear2;
    private final SparkMaxAnalogSensor a_linear1;
    private final SparkMaxAnalogSensor a_linear2;
    private final SparkMaxPIDController p_linear1;
    private final SparkMaxPIDController p_linear2; 


    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable;


    private boolean linearInitialized = false;
    private boolean linearInitializedAutoDigging = false;

    private LinearActuatorStateRight linearStateRight = LinearActuatorStateRight.Unknown;
    private LinearActuatorStateLeft linearStateLeft = LinearActuatorStateLeft.Unknown;


    private Runnable linearRunnable;
    private Notifier linearNotifier;


    private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
    private HashMap<String, GenericEntry> pidNTEntries = new HashMap<String, GenericEntry>();
    private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();


    public DiggingUppies() {
        m_linear1 = new LunaSparkMax(LINEAR_2_CAN_ID, MotorType.kBrushed);
        a_linear1 = m_linear1.getAnalogSensor();
        p_linear1 = m_linear1.getPIDController();


        m_linear2 = new LunaSparkMax(LINEAR_3_CAN_ID, MotorType.kBrushed);
        a_linear2 = m_linear2.getAnalogSensor();
        p_linear2 = m_linear2.getPIDController();


        m_linear1.burnFlash();
        m_linear2.burnFlash();


        // m_linear2.follow(m_linear1, false);




        networkTable = ntInstance.getTable("DiggingSubsystem");


        createDashboardData();


        // Make sure linear actuator relays are in off state
        linearStopRight();
        linearStopLeft();




        if (ENABLE_TEST_DASHBOARDS){
            reportInitialPID();
            pidConstants.put("digging-actuator-kP", DIGGING_LINEAR_kP);
            pidConstants.put("digging-actuator-kI", DIGGING_LINEAR_kI);
            pidConstants.put("digging-actuator-kD", DIGGING_LINEAR_kD);
            pidConstants.put("digging-actuator-kIZ", DIGGING_LINEAR_kIZ);
            pidConstants.put("digging-actuator-kFF", DIGGING_LINEAR_kFF);
            pidConstants.put("digging-actuator-setpoint", 0.0);
        }
    }


    private void createDashboardData() {
        
        shuffleboardEntries.put("digging-actuator-init", Shuffleboard.getTab("Competition").add("Digging Linear Init", linearInitialized)
            .withSize(1, 1).withPosition(3, 0).getEntry());
        shuffleboardEntries.put("digging-actuator-state", Shuffleboard.getTab("Competition")
            .add("Digging Linear State", linearStateRight.toString()).withSize(1, 1).withPosition(3, 1).getEntry());
        shuffleboardEntries.put("digging-actuator-position1",
            Shuffleboard.getTab("Digging Actuator PID").add("Digging Actuator Position1", 0).withSize(1, 1).withPosition(3, 2).getEntry());
            shuffleboardEntries.put("digging-actuator-position2",
            Shuffleboard.getTab("Digging Actuator PID").add("Digging Actuator Position2", 0).withSize(1, 1).withPosition(3, 3).getEntry());
        shuffleboardEntries.put("digging-position-command",
            Shuffleboard.getTab("Competition").getLayout("LACommand", BuiltInLayouts.kList).withSize(1, 2)
                .withPosition(3, 4).add("Digging Linear Position", 0.0).getEntry());
        
   }


    @Override
    public void periodic() {
        // This will be called once per scheduler run
        reportSensors(); 
        checkLimits();
        if (ENABLE_TEST_DASHBOARDS)
            checkPIDGains();
       // System.out.println("DIGGING ACTUATOR 1 POSITION: " + a_linear1.getPosition());
       // System.out.println("DIGGING ACTUATOR 2 POSITION: " + a_linear2.getPosition());
    }


    private void linearUp() {
        if (a_linear1.getPosition() >= (LINEAR_MAX_TRAVEL2 - LINEAR_DEADBAND) || linearStateRight == LinearActuatorStateRight.Lowered) {
            m_linear1.stopMotor();
        
            return;
        }
        if (a_linear2.getPosition() >= (LINEAR_MAX_TRAVEL2 - LINEAR_DEADBAND) || linearStateLeft == LinearActuatorStateLeft.Lowered) {
            m_linear2.stopMotor();
        
            return;
        }


        linearStateRight = LinearActuatorStateRight.TravelingUp;
        linearStateLeft = LinearActuatorStateLeft.TravelingUp;


        m_linear1.set(-1);
        m_linear2.set(-1);
    }


    private void linearDown() {
        if (a_linear1.getPosition() <= LINEAR_MIN_TRAVEL
                || linearStateRight == LinearActuatorStateRight.Lowered)
            return;
        if (a_linear2.getPosition() <= LINEAR_MIN_TRAVEL
                || linearStateLeft == LinearActuatorStateLeft.Lowered)
            return;
        linearStateRight = LinearActuatorStateRight.TravelingDown;
        linearStateLeft = LinearActuatorStateLeft.TravelingDown;


        m_linear1.set(1);
        m_linear2.set(1);
    }


    private void linearStopRight() {
        m_linear1.set(0);
    }
    private void linearStopLeft() {
        m_linear2.set(0);
    }


    public void linearActuatorInitStart() {
        linearDown();
        // linearUp();
    }

    public void linearActuatorInitStartAuto()
    {
        linearUp();
    }



    public double linearActuatorGetRawPotentiometer() {
        return a_linear1.getVoltage();
    }


    public void linearActuatorInitEnd() {
        linearStopRight();
        linearStopLeft();
        linearRunnable = new LinearActuatorLimits();
        linearNotifier = new Notifier(linearRunnable);
        linearNotifier.startPeriodic(0.005);
        linearInitialized = true;
    }

    public void linearActuatorInitEndAutoDigging() {
        linearStopRight();
        linearStopLeft();
        linearRunnable = new LinearActuatorLimits();
        linearNotifier = new Notifier(linearRunnable);
        linearNotifier.startPeriodic(0.005);
        linearInitializedAutoDigging = true;
    }

    public void linearStopMoving()
    {
        linearStopLeft();
        linearStopRight();
    }


    public boolean isLinearActuatorInitialized() {
        return linearInitialized;
    }

    public boolean isLinearActuatorInitializedAutoDigging() {
        return linearInitializedAutoDigging;
    }


    public void linearActuatorRight(LinearActuatorStateRight state) {
        if (state == LinearActuatorStateRight.Raised) {
            linearUp();
        } else if (state == LinearActuatorStateRight.Lowered) {
            linearDown();
        }
    }


    public void linearActuatorLeft(LinearActuatorStateLeft state) {
        if (state == LinearActuatorStateLeft.Raised) {
            linearUp();
        } else if (state == LinearActuatorStateLeft.Lowered) {
            linearDown();
        }
    }


    public LinearActuatorStateRight linearActuatorStateRight() {
        return linearStateRight;
    }


    public LinearActuatorStateLeft linearActuatorStateLeft() {
        return linearStateLeft;
    }




    public double getDashboardCommandedPosition() {
        return MathUtil.clamp(shuffleboardEntries.get("digging-position-command").getDouble(LINEAR_MIN_TRAVEL),
                LINEAR_MIN_TRAVEL, LINEAR_MAX_TRAVEL2);
    }


    public double getLinearPositionRight() 
    {
        return a_linear1.getPosition();
    }


    public double getLinearPositionLeft() 
    {
        return a_linear2.getPosition();
    }


    public void commandDown(){
        linearDown();
        linearStateRight = LinearActuatorStateRight.Commanded;
        linearStateLeft = LinearActuatorStateLeft.Commanded;


    }


    public void commandUp(){
        linearUp();
        linearStateRight = LinearActuatorStateRight.Commanded;
        linearStateLeft = LinearActuatorStateLeft.Commanded;


    }


    public void commandStop(){
        linearStopRight();
        linearStopLeft();
    }


    private void checkPIDGains() {
        for (String pidConstant : pidConstants.keySet()) {
            double newVal = pidNTEntries.get(pidConstant).getDouble(pidConstants.get(pidConstant));
            if (newVal == pidConstants.get(pidConstant))
                continue;
            System.out.println("LS: " + pidConstant + " value updated: " + newVal);
            String[] split = pidConstant.split("-", 2);
            System.out.println("Split: " + split[1]);
                switch (split[1]) {
                    case "actuator-kP":
                        p_linear1.setP(newVal);
                        break;
                    case "actuator-kI":
                        p_linear1.setI(newVal);
                        break;
                    case "actuator-kD":
                        p_linear1.setD(newVal);
                        break;
                    case "actuator-kIZ":
                        p_linear1.setIZone(newVal);
                        break;
                    case "actuator-kFF":
                        p_linear1.setFF(newVal);
                        break;
                    case "actuator-setpoint":
                        p_linear1.setReference(newVal, ControlType.kPosition);
                        break;
                    default:
                        break;
                }
            pidConstants.put(pidConstant, newVal);
        }
    }


    private void reportSensors() {
        shuffleboardEntries.get("digging-actuator-init").setBoolean(linearInitialized);
        shuffleboardEntries.get("digging-actuator-state").setString(linearStateRight.toString());
        shuffleboardEntries.get("digging-actuator-position1").setDouble(LunaMathUtils.roundToPlace(a_linear1.getPosition(), 3));
        shuffleboardEntries.get("digging-actuator-position2").setDouble(LunaMathUtils.roundToPlace(a_linear2.getPosition(), 3));
        networkTable.getEntry("diggingLinearInitialized").setBoolean(linearInitialized);
        networkTable.getEntry("diggingLinearPosition").setDouble(a_linear1.getPosition());
        networkTable.getEntry("diggingLinearState").setString(linearStateRight.toString());
    }


    private void reportInitialPID(){
        pidNTEntries.put("digging-actuator-kP",
             Shuffleboard.getTab("Digging Actuator PID").add("Digging Actuator P Gain", DIGGING_LINEAR_kP).getEntry());
        pidNTEntries.put("digging-actuator-kI", 
            Shuffleboard.getTab("Digging Actuator PID").add("Digging Actuator I Gain", DIGGING_LINEAR_kI).getEntry());
        pidNTEntries.put("digging-actuator-kD", 
            Shuffleboard.getTab("Digging Actuator PID").add("Digging Actuator D Gain", DIGGING_LINEAR_kD).getEntry());
        pidNTEntries.put("digging-actuator-kIZ",
             Shuffleboard.getTab("Digging Actuator PID").add("Digging Actuator I Zone", DIGGING_LINEAR_kIZ).getEntry());
        pidNTEntries.put("digging-actuator-kFF", 
            Shuffleboard.getTab("Digging Actuator PID").add("Digging Actuator FF", DIGGING_LINEAR_kFF).getEntry());
        pidNTEntries.put("digging-actuator-setpoint", 
            Shuffleboard.getTab("Digging Actuator PID").add("Digging Actuator Setpoint", 0).getEntry());
        pidNTEntries.put("digging-actuator-velocity", 
            Shuffleboard.getTab("Digging Actuator PID").add("Digging Actuator Velocity", 0).getEntry());
    }


    private void checkLimits() {
        // System.out.println("Current Position: " + a_linear1.getPosition());
        if (linearStateRight != LinearActuatorStateRight.Raised && linearStateRight != LinearActuatorStateRight.TravelingDown
                && a_linear1.getPosition() >= (LINEAR_MAX_TRAVEL2 - LINEAR_DEADBAND)) {
            linearStateRight = LinearActuatorStateRight.Raised;
            linearStopRight();
        }
        if (linearStateRight != LinearActuatorStateRight.Lowered && linearStateRight != LinearActuatorStateRight.TravelingUp
                && a_linear1.getPosition() <= LINEAR_MIN_TRAVEL) {
            linearStateRight = LinearActuatorStateRight.Lowered;
            linearStopRight();
        }
        if (linearStateLeft != LinearActuatorStateLeft.Raised && linearStateLeft != LinearActuatorStateLeft.TravelingDown
                && a_linear2.getPosition() >= (LINEAR_MAX_TRAVEL2 - LINEAR_DEADBAND)) {
            linearStateLeft = LinearActuatorStateLeft.Raised;
            linearStopLeft();
        }
        if (linearStateLeft != LinearActuatorStateLeft.Lowered && linearStateLeft != LinearActuatorStateLeft.TravelingUp
                && a_linear2.getPosition() <= LINEAR_MIN_TRAVEL) {
            linearStateLeft = LinearActuatorStateLeft.Lowered;
            linearStopLeft();
        }
    }


    public class LinearActuatorLimits implements Runnable {


        @Override
        public void run() {
            checkLimits();
        }


    }
}