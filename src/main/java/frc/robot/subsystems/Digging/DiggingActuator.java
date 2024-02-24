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



public class DiggingActuator {

    private final LunaSparkMax m_linear1;
    private final SparkMaxAnalogSensor a_linear;
    private final SparkMaxPIDController p_linear;

    public DiggingActuator(){
        m_linear1 = new LunaSparkMax(LINEAR_1_CAN_ID, MotorType.kBrushed);
        a_linear = m_linear1.getAnalogSensor();
        p_linear = m_linear1.getPIDController();
    }
    
}
