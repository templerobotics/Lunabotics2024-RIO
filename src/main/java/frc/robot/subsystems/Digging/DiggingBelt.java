package frc.robot.subsystems.Digging;


import com.revrobotics.RelativeEncoder; //replaced CANEncoder
import com.revrobotics.SparkMaxPIDController; //replaced CANPIDController
import com.revrobotics.CANSparkMax.ControlType; //this is the new one. may need to install "npm install com.revrobotics.CANSparkMax"
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.custom.LunaSparkMax;
import frc.robot.custom.LunaSparkMax.Presets;
import frc.robot.custom.LunaMathUtils;

import static frc.robot.Constants.DiggingConstants.*;
import static frc.robot.Constants.GlobalConstants.*;

import java.util.HashMap;
import java.util.Map;



public class DiggingBelt extends SubsystemBase {

    private final LunaSparkMax m_belt1;
    private final LunaSparkMax m_belt2;
    private final RelativeEncoder e_belt;
    private final SparkMaxPIDController p_belt;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable;

    private static boolean beltRunning;
    private int beltSpeed = 11000;

    //Hash Map for PID constants
    private HashMap<String, Double> pidConstants = new HashMap<String, Double>();
    //Hash Map for Network Table
    private HashMap<String, GenericEntry> pidNTEntries = new HashMap<String, GenericEntry>();
    //Hash Map for Shuffleboard
    private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();

    public DiggingBelt(){
        //Instantiate the belts
        m_belt1 = new LunaSparkMax(BELT_1_CAN_ID, MotorType.kBrushless, Presets.kDiggingBelt);
        m_belt2 = new LunaSparkMax(BELT_2_CAN_ID, MotorType.kBrushless, Presets.kDiggingBelt);

        e_belt = m_belt1.getEncoder();
        p_belt = m_belt1.getPIDController();

        networkTable = ntInstance.getTable("Digging Subsystem");

        m_belt2.follow(m_belt1, true);

        m_belt1.burnFlash();
        m_belt2.burnFlash();

        createDashboardData();

        if (ENABLE_TEST_DASHBOARDS){
            reportInitialPID();
            pidConstants.put("belt-kP", LEADSCREW_kP);
            pidConstants.put("belt-kI", LEADSCREW_kI);
            pidConstants.put("belt-kD", LEADSCREW_kD);
            pidConstants.put("belt-kIZ", LEADSCREW_kIZ);
            pidConstants.put("belt-kFF", LEADSCREW_kFF);
            pidConstants.put("belt-setpoint", 0.0);
        }
    }

    @Override
    public void periodic(){
        //This will be called once per scheduler run
        reportSensors();
        dashboardBeltSpeed();
        if (ENABLE_TEST_DASHBOARDS){
            checkPIDGains();
        }
    }

    private void createDashboardData(){
        shuffleboardEntries.put("belt-run", Shuffleboard.getTab("Competition").add("Dig Belt Run", false).withSize(1,1).withPosition(9,0).getEntry());
        shuffleboardEntries.put("belt-units", Shuffleboard.getTab("Competition").add("Dig Belt Units", "RPM").withSize(1, 1).withPosition(9, 1).getEntry());
		shuffleboardEntries.put("belt-speed", Shuffleboard.getTab("Competition").add("Real Speed", 0).withSize(1, 1).withPosition(9, 2).getEntry());
		shuffleboardEntries.put("belt-speed-slider", Shuffleboard.getTab("Competition").add("Command Speed", beltSpeed).withSize(3, 1).withPosition(8, 3).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 11000)).getEntry());
		shuffleboardEntries.put("motor-temperature-1", Shuffleboard.getTab("Competition").add("Leader Temperature", m_belt1.getMotorTemperature() + " C").withSize(1,1).withPosition(9,4).getEntry());
		shuffleboardEntries.put("motor-temperature-2", Shuffleboard.getTab("Competition").add("Follower Temperature", m_belt2.getMotorTemperature() + " C").withSize(1,1).withPosition(10,4).getEntry());
    }

    //Set belt speed and update dashboard
    public void setBeltSpeed(double speed){
        m_belt1.set(speed);
        beltRunning = true;
        networkTable.getEntry("beltSetPoint").setDouble(speed);
        shuffleboardEntries.get("belt-run").setBoolean(true);
    }

    public void runBelt(boolean reverse){
        int tempSpeed = beltSpeed;
        if (reverse){
            tempSpeed = tempSpeed * -1;
        }
        p_belt.setReference(tempSpeed, ControlType.kVelocity);
        beltRunning = true;
        networkTable.getEntry("beltSetpoint").setDouble(beltSpeed);
        shuffleboardEntries.get("belt-run").setBoolean(true);
    }

    public void stopBelt(){
        m_belt1.stopMotor();
        beltRunning = false;
        networkTable.getEntry("beltSetpoint").setDouble(0);
        shuffleboardEntries.get("belt-run").setBoolean(false);
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

    private void checkPIDGains(){
        //Itterate through each PID Const.
        for (String pidConstant : pidConstants.keySet()){
            //Get current PID gain values
            double newVal = pidNTEntries.get(pidConstant).getDouble(pidConstants.get(pidConstant));
            //Check if value has changed
            if(newVal == pidConstants.get(pidConstant)){
                continue;
            }
            //Get type of PID gain constant
            String[] split = pidConstant.split("-", 2);

            //Update the corresponding PID parameter in the p_belt object
            switch(split[1]){
                case "kP":
                    p_belt.setP(newVal);
                    break;
                case "kI":
                    p_belt.setI(newVal);
                    break;
                case "kD":
                    p_belt.setD(newVal);
                    break;
                case "kIZ":
                    p_belt.setIZone(newVal);
                    break;
                case "kFF":
                    p_belt.setFF(newVal);
                    break;
                case "setpoint":
                    //Update setpoint and turn belt off if needed
                    if (newVal == 0){
                        beltRunning = false;
                    }else{
                        beltRunning = true;
                    }
                    p_belt.setReference(newVal, ControlType.kVelocity);
                    break;
                default:
                    break;
            }
            //Update values stored in pidConstants
            pidConstants.put(pidConstant, newVal);

        }
        //Set current velocity of the velt in Network Table Entry
        pidNTEntries.get("belt-velocity").setDouble(e_belt.getVelocity());
    }

    private void dashboardBeltSpeed(){
        int newVal = (int) shuffleboardEntries.get("belt-speed-slider").getDouble(beltSpeed);
        if (newVal == beltSpeed){
            return;
        }
        beltSpeed = newVal;
    }

    private void reportSensors(){
        //Determine units for belt velocity
        boolean unitFlag = Double.compare(BELT_VELOCITY_SCALAR, 1.0) == 0;

        //Update shuffleboard entries
        shuffleboardEntries.get("belt-units").setString(unitFlag ? "RPM" : "m/s");
        shuffleboardEntries.get("belt-speed").setDouble(LunaMathUtils.roundToPlace(e_belt.getVelocity(), 2));

        //Report motor temperatures to Shuffleboard
        shuffleboardEntries.get("motor-temperature-1").setString(""+LunaMathUtils.roundToPlace(m_belt1.getMotorTemperature(), 5));
        shuffleboardEntries.get("motor-temperature-2").setString(""+LunaMathUtils.roundToPlace(m_belt2.getMotorTemperature(), 5));

        //Update Network Table Entries
        networkTable.getEntry("beltVelocity").setDouble(e_belt.getVelocity());
        networkTable.getEntry("belt1Current").setDouble(m_belt1.getOutputCurrent());
        networkTable.getEntry("belt2Current").setDouble(m_belt2.getOutputCurrent());
    }

    public static boolean isBeltRunning(){
        return beltRunning;
    }

} 