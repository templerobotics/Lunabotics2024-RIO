package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.custom.*;


public class GearServo extends SubsystemBase
{
    private PWM gearServo;
    public int count = 0;




 // NetworkTable Instantiation
 private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
 private final NetworkTable networkTable;




    public GearServo()
    {
        gearServo = new Servo(1);
        networkTable = ntInstance.getTable("DumpingSubsystem");
    }
    
    
    @Override
    public void periodic() {
        // System.out.println("Gear Angle: " + getGearAngle()); 
    }




    public void servoClockwise() {
        setRopeSpeed(1.0);
    }

    public void servoCClockwise() {
        setRopeSpeed(-1.0);
    }

    public double getRopeSpeed() {
        return gearServo.getSpeed();
    }

    public void setRopeSpeed(double speed) {
        gearServo.setSpeed(speed);
    }
    public void stopRope() {
        gearServo.setRaw(0);
    }

    public double getRopePosition() {
        double gearPosition = LunaMathUtils.scaleBetween(gearServo.getPosition(), 0.0, 1.0, 0.0, 360.0);
        return gearPosition;
    }

    public double getCount() {
        return count;
    }

    public void resetCount() {
        count = 0;
    }
}

