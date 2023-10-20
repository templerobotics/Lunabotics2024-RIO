package frc.robot.custom;
import frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import frc.robot.custom.LunaSparkMax.Presets;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import static frc.robot.Constants.SparkMaxConsants.*;

public class LunaSparkTest extends CANSparkMax{
    private final Presets preset;
    private final int can_id;
    private RelativeEncoder encoder;
    private SparkMaxPIDController pid;


    /**
     * Constructs an LunaSparkMax
     * 
     * @param can_id Spark Max's CAN ID
     * @param m_type The type of motor connected to the Spark Max
     * @param preset The Motor Preset this object will utilize
     */
    public LunaSparkTest (int can_id, MotorType m_type, Presets preset) {
        super(can_id, m_type);
        this.preset = preset;
        this.can_id = can_id;
    }

    /**
     * 
     * @param can_id Spark Max's CAN ID
     * @param m_type The type of motor connected to the Spark Max
     */
    public LunaSparkTest(int can_id, MotorType m_type) {
        super(can_id, m_type);
        this.preset = Presets.kNone;
        this.can_id = can_id;
    }
}
