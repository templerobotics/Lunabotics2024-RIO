package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;

import frc.robot.custom.*;

public class DumpServo extends SubsystemBase {
    private PWM ropeServo;
    public int count = 0;


    // NetworkTable Instantiation
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable networkTable;

    public DumpServo () {
        ropeServo = new PWM(0);
        // ropeServo.setPosition(0);
        networkTable = ntInstance.getTable("DumpingSubsystem");
    }

    @Override
    public void periodic() {
        //System.out.println("Rope Position: " + getRopePosition());  
    }

    public void servoClockwise() {
        setRopeSpeed(1.0);
    }

    public void servoCClockwise() {
        setRopeSpeed(-1.0);
    }

    public double getRopeSpeed() {
        return ropeServo.getSpeed();
    }

    public void setRopeSpeed(double speed) {
        ropeServo.setSpeed(speed);
    }
    public void stopRope() {
        ropeServo.setRaw(0);
    }

    public double getRopePosition() {
        double ropePosition = LunaMathUtils.scaleBetween(ropeServo.getPosition(), 0.0, 1.0, 0.0, 360.0);
        return ropePosition;
    }

    public double getCount() {
        return count;
    }

    public void resetCount() {
        count = 0;
    }
}


