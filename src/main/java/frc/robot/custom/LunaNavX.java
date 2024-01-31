package frc.robot.custom;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

// RMC Rules state we cannot use any magnetic-based sensors
// So we wrap the navX methods to make sure we only use accelerometer and gyroscope
public class LunaNavX {
    private final AHRS ahrs;
    
    public LunaNavX() {
        ahrs = new AHRS(SPI.Port.kMXP);
    }

    public void calibrate() {
        ahrs.calibrate();
    }

    public float getAccelX() {
        return ahrs.getRawAccelX();
    }

    public float getAccelY(){
		return ahrs.getRawAccelY();
	}

	public float getAccelZ(){
		return ahrs.getRawAccelZ();
	}

	public float getGyroX(){
		return ahrs.getRawGyroX();
	}

	public float getGyroY(){
		return ahrs.getRawGyroY();
	}

	public float getGyroZ(){
		return ahrs.getRawGyroZ();
	}

	public boolean isMoving(){
		return ahrs.isMoving();
	}

	public void reset(){
		ahrs.reset();
	}
}
