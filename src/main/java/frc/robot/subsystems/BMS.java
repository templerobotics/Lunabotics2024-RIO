package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BMS extends SubsystemBase{
    SerialPort arduinoBMS;

    
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable;
    private HashMap<String, GenericEntry> shuffleboardEntries = new HashMap<String, GenericEntry>();
    
    public BMS() {
        arduinoBMS = new SerialPort(115200, SerialPort.Port.kMXP);
        networkTable = ntInstance.getTable("BMS");
        
        createDashBoardData();
    }

    @Override
    public void periodic() {
        parseArduinoOutput();
    }

    private void createDashBoardData() {
        for (int i = 1; i <= 12; i++) {
            String cellKey = "cell-" + i;
            String cellLabel = "Cell " + i + " Voltage";
            shuffleboardEntries.put(cellKey, Shuffleboard.getTab("BMS").add(cellLabel, 0).withSize(1, 1).withPosition((i - 1) % 4, (i - 1) / 4).getEntry());
        }
    }
    
    public void parseArduinoOutput() {
        String arduinoOutput = arduinoBMS.readString();
        String[] cellVoltages = arduinoOutput.trim().split("\\s+");

        for (int i = 0; i < cellVoltages.length; i++) {
            String cellName = "cell-" + (i + 1);
            double voltage = Double.parseDouble(cellVoltages[i]);
            System.out.println(cellName + ": " + voltage);

            // Update NetworkTable and Shuffleboard
            networkTable.getEntry(cellName).setDouble(voltage);
            shuffleboardEntries.get(cellName).setDouble(voltage);
        }

    }
}