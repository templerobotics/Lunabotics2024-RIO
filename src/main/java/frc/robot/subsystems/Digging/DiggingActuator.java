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
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.custom.LunaSparkMax;
import static frc.robot.Constants.GlobalConstants.*;

import java.util.HashMap;

/*
    NOTE --> Left = Actuator1 & Right = Actuator 2
*/

public class DiggingActuator extends SubsystemBase{

    private final LunaSparkMax m_linear1;
    private final LunaSparkMax m_linear2;
    private final SparkMaxAnalogSensor a_linear;
    private final SparkMaxAnalogSensor a_linear2;
    
    private final SparkMaxPIDController p_linear;
    private final SparkMaxPIDController p_linear2;

    private LinearActuatorState linearState_dig = LinearActuatorState.Unknown;
    private boolean linearInitialized = false;
    private boolean linearInitialized_2 = false;
    
    public enum LinearActuatorState {
		Unknown, Raised, Lowered, TravelingUp, TravelingDown, Commanded;
	}
 
    //TO:DO implement
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
	private final NetworkTable networkTable;

    private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
	private HashMap<String, GenericEntry> pidNTEntries = new HashMap<String, GenericEntry>();
	private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();
    
    //TO DO : Utilize this variable
    private SparkMaxPIDController[] pidControllers;
    
    private Runnable linearRunnable;
    private Notifier linearNotifier;

    public DiggingActuator(){
        m_linear1 = new LunaSparkMax(LINEAR_1_CAN_ID, MotorType.kBrushed);
        m_linear2 = new LunaSparkMax(LINEAR_2_CAN_ID, MotorType.kBrushed);

        a_linear = m_linear1.getAnalogSensor();
        a_linear2 = m_linear2.getAnalogSensor();

        p_linear = m_linear1.getPIDController();
        p_linear2 = m_linear2.getPIDController();
        
        //TO DO : Implement Network Table key,values pairs once vision of how we use network tables is clear
        networkTable = ntInstance.getTable("DiggingSubsystem");
        createDashBoardData();
        linearStop();
        
        if (ENABLE_TEST_DASHBOARDS){
            reportInitialPID();
            pidConstants.put("digging-kP", LINEAR_kP);
            pidConstants.put("digging-kI", LINEAR_kI);
            pidConstants.put("digging-kD", LINEAR_kD);
            pidConstants.put("digging-kIZ", LINEAR_kIZ);
            pidConstants.put("digging-kFF", LINEAR_kFF);
            pidConstants.put("digging-setpoint", 0.0);
        }

    }

    private void createDashBoardData() {
        shuffleboardEntries.put("digging-init-LEFT", Shuffleboard.getTab("Competition").add("Linear Init-LEFT", linearInitialized)
			.withSize(1, 1).withPosition(3, 0).getEntry());
        shuffleboardEntries.put("digging-init-RIGHT", Shuffleboard.getTab("Competition").add("Linear Init-RIGHT", linearInitialized_2)
			.withSize(1, 1).withPosition(4, 0).getEntry());
        
		shuffleboardEntries.put("digging-state", Shuffleboard.getTab("Competition")
			.add("Linear State", linearState_dig.toString()).withSize(1, 1).withPosition(3, 1).getEntry());

        shuffleboardEntries.put("digging-position-LEFT",Shuffleboard.getTab("Competition").add("Digging Position-LEFT", 0).withSize(1, 1).withPosition(3, 2).getEntry());
        shuffleboardEntries.put("digging-position-RIGHT",Shuffleboard.getTab("Competition").add("Digging Position-RIGHT", 0).withSize(1, 1).withPosition(4, 2).getEntry());
        
        shuffleboardEntries.put("position-command",
            Shuffleboard.getTab("Competition").getLayout("LACommand", BuiltInLayouts.kList).withSize(1, 2)
                .withPosition(3, 4).add("Position", 0.0).getEntry());
    }

    private void reportSensors() {
		shuffleboardEntries.get("digging-init-LEFT").setBoolean(linearInitialized);
        shuffleboardEntries.get("digging-init-RIGHT").setBoolean(linearInitialized_2);
		shuffleboardEntries.get("digging-state").setString(linearState_dig.toString());
        shuffleboardEntries.get("digging-position-LEFT").setDouble(LunaMathUtils.roundToPlace(a_linear.getPosition(), 3));
        shuffleboardEntries.get("digging-position-RIGHT").setDouble(LunaMathUtils.roundToPlace(a_linear2.getPosition(), 3));
        //Network Tables subscribers waiting for publishers
		networkTable.getEntry("diggit-init-LEFT").setBoolean(linearInitialized);
        networkTable.getEntry("digging-init-RIGHT").setBoolean(linearInitialized_2);
        networkTable.getEntry("digging-position-LEFT").setDouble(a_linear.getPosition());
        networkTable.getEntry("digging-position-RIGHT").setDouble(a_linear2.getPosition());
		networkTable.getEntry("linearState_dig").setString(linearState_dig.toString());
	}

    private void reportInitialPID(){
        pidNTEntries.put("digging-kP", Shuffleboard.getTab("Digging PID").add("Digging P Gain", LINEAR_kP).getEntry());
		pidNTEntries.put("digging-kI", Shuffleboard.getTab("Digging PID").add("Digging I Gain", LINEAR_kI).getEntry());
		pidNTEntries.put("digging-kD", Shuffleboard.getTab("Digging PID").add("Digging D Gain", LINEAR_kD).getEntry());
		pidNTEntries.put("digging-kIZ", Shuffleboard.getTab("Digging PID").add("Digging I Zone", LINEAR_kIZ).getEntry());
		pidNTEntries.put("digging-kFF", Shuffleboard.getTab("Digging PID").add("Digging FF", LINEAR_kFF).getEntry());
		pidNTEntries.put("digging-setpoint", Shuffleboard.getTab("Digging PID").add("Digging Setpoint", 0).getEntry());
        //Method below overwrites default velocity value of 0 --> Should be fine(Pre-Testing Idea)
		pidNTEntries.put("digging-position-LEFT", Shuffleboard.getTab("Digging PID").add("Digging Position-LEFT", 0).getEntry());
        pidNTEntries.put("digging-position-RIGHT", Shuffleboard.getTab("Digging PID").add("Digging Position-RIGHT", 0).getEntry());
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
                        p_linear2.setP(newVal);
                        break;
                    case "kI":
                        p_linear.setI(newVal);
                        p_linear2.setI(newVal);
                        break;
                    case "kD":
                        p_linear.setD(newVal);
                        p_linear2.setD(newVal);
                        break;
                    case "kIZ":
                        p_linear.setIZone(newVal);
                        p_linear2.setIZone(newVal);
                        break;
                    case "kFF":
                        p_linear.setFF(newVal);
                        p_linear2.setFF(newVal);
                        break;
                    case "setpoint":
                        p_linear.setReference(newVal, ControlType.kPosition);
                        p_linear2.setReference(newVal, ControlType.kPosition);
                        break;
                    default:
                        break;
                }
			pidConstants.put(pidConstant, newVal);
		}
		pidNTEntries.get("Digging Position-LEFT").setDouble(a_linear.getPosition());
        pidNTEntries.get("Digging Position-RIGHT").setDouble(a_linear2.getPosition());
	}

    public double getDashboardCommandedPosition() {
		return MathUtil.clamp(shuffleboardEntries.get("position-command").getDouble(LINEAR_MIN_TRAVEL),
				LINEAR_MIN_TRAVEL, LINEAR_MAX_TRAVEL);
	}
    
    private void linearUp(){
        if (a_linear.getPosition() >= 1  || a_linear2.getPosition()>=1|| linearState_dig == LinearActuatorState.Lowered) {
            m_linear1.stopMotor();
            return;
        }
        linearState_dig = LinearActuatorState.TravelingDown;
        m_linear1.set(-1);
    }

     /*
        Dumping conditions are "<=" Oppsite of "<=" = ">"
    */
    private void linearDown(){
        assert(a_linear.getPosition()>LINEAR_MIN_TRAVEL && a_linear2.getPosition()>LINEAR_MIN_TRAVEL && linearState_dig != LinearActuatorState.Lowered);
        linearState_dig = LinearActuatorState.TravelingDown;
        m_linear1.set(1);
        m_linear2.set(1);
    }
    
    private void linearStop() {
        m_linear1.set(0);
        m_linear2.set(0);
     }

    public void linearActuatorInitStart() {
        linearDown();
    }

    public LinearActuatorState getlinearActuatorState() {
        return linearState_dig;
    }

    public double getLinearPosition(RobotSide side){
		if (side == RobotSide.Left) return ((MotorController) a_linear).get();
		else return ((MotorController) a_linear2).get()+LINEAR_2_ADJUSTMENT;
	}
/* 
    public boolean isLinearActuatorInitializedLEFT() {
        return linearInitialized;
    }
    public boolean isLinearActuatorInitializedRIGHT(){
        return linearInitialized_2;
    }

    public double getLinearActuatorPositionLeft() {
        return a_linear.getPosition();
    }

    public double getLinearActuatorPositionRight(){
        return a_linear2.getPosition();
    }
*/
    public double getLinearActuatorPosition(SparkMaxAnalogSensor sparkmax_actuator){
        return sparkmax_actuator.getPosition();
    }

    public boolean isLinearActuatorInitialized(boolean linear_initialized){
        return linear_initialized;
    }

    public double getlinearActuatorGetRawPotentiometer(SparkMaxAnalogSensor sparkmax_actuator) {   
        return  sparkmax_actuator.getVoltage();
    }

    private void checkLinearActuatorLimits(){
        if (linearState_dig != LinearActuatorState.Raised && linearState_dig != LinearActuatorState.TravelingDown && a_linear.getPosition() >= LINEAR_MAX_TRAVEL - LINEAR_DEADBAND && a_linear2.getPosition()>=LINEAR_MAX_TRAVEL-LINEAR_DEADBAND) {
            linearState_dig = LinearActuatorState.Raised;
            linearStop();
        }
        if (linearState_dig != LinearActuatorState.Lowered && linearState_dig != LinearActuatorState.TravelingUp && a_linear.getPosition() <= LINEAR_MIN_TRAVEL && a_linear2.getPosition()<= LINEAR_MIN_TRAVEL) {
            linearState_dig = LinearActuatorState.Lowered;
            linearStop();
        }
    }

    public void commandDown(){
		linearDown();
		linearState_dig = LinearActuatorState.Commanded;
	}

	public void commandUp(){
		linearUp();
		linearState_dig = LinearActuatorState.Commanded;
	}

	public void commandStop(){
		linearStop();
	}

    public void linearActuatorInitEnd() {
		linearStop();
		linearRunnable = new LinearActuatorLimits();
		linearNotifier = new Notifier(linearRunnable);
		linearNotifier.startPeriodic(0.005);
		linearInitialized = true;
        linearInitialized_2 = true;
	}

    public void linearActuatorDigging(LinearActuatorState state) {
		if (state == LinearActuatorState.Raised) {
			linearUp();
		} else if (state == LinearActuatorState.Lowered) {
			linearDown();
		}
	}

    @Override
    public void periodic(){
         // This will be called once per scheduler run
         reportSensors(); 
         checkLinearActuatorLimits();
         // checkPIDGains();
 
        double linearP = SmartDashboard.getNumber("linearP", 0.0);
        double linearI = SmartDashboard.getNumber("linearI", 0.0);
        double linearD = SmartDashboard.getNumber("linearD", 0.0);
        double linearFF = SmartDashboard.getNumber("linearFF", 0.0);
        double setPt = SmartDashboard.getNumber("Target Position", 0.0);
 
        if (linearP != p_linear.getP() && linearP !=p_linear2.getP()){p_linear.setP(linearP); p_linear2.setP(linearP);}
        if (linearP != p_linear.getI()&& linearP !=p_linear2.getI() ) {p_linear.setI(linearI); p_linear2.setI(linearI);}
        if (linearP != p_linear.getD()&& linearP !=p_linear2.getD()) {p_linear.setD(linearD); p_linear2.setD(linearD);}
        if (linearP != p_linear.getFF()&& linearP !=p_linear2.getFF()) {p_linear.setFF(linearFF); p_linear2.setFF(linearFF);}
        
        SmartDashboard.putNumber("linearP-LEFT", p_linear.getP());
        SmartDashboard.putNumber("linearP-RIGHT", p_linear2.getP());
        
        SmartDashboard.putNumber("linearI-LEFT", p_linear.getI());
        SmartDashboard.putNumber("linearI-RIGHT", p_linear2.getI());

        SmartDashboard.putNumber("linearD-LEFT", p_linear2.getD());
        SmartDashboard.putNumber("linearD-RIGHT", p_linear2.getD());

        SmartDashboard.putNumber("linearFF-LEFT", p_linear.getFF());
        SmartDashboard.putNumber("linearFF-RIGHT", p_linear2.getFF());

        SmartDashboard.putNumber("Target Position", setPt);

        SmartDashboard.putNumber("Current Position-LEFT", a_linear.getPosition());   
        SmartDashboard.putNumber("Current Position-RIGHT", a_linear2.getPosition());   
    }

    public class LinearActuatorLimits implements Runnable {
		@Override
		public void run() {checkLinearActuatorLimits();}
	}


}
