package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

public class Constants {

    public final static class ControllerConstants {
        public final static int XBOX_CONTROLLER_DRIVER = 0;
        
    }
    
    public final static class MathConstants {
        public final static double RPM_TO_MPS = 2*Math.PI/60;
    }

    public final static class SparkMaxConsants {
        public final static double DEFAULT_OPEN_LOOP_RAMP_RATE = 1;
        public final static IdleMode DEFAULT_IDLE_MODE = IdleMode.kCoast;
        public final static int DEFAULT_SMART_CURRENT_LIMIT = 5;
        public final static double DEFAULT_SECONDARY_CURRENT_LIMIT = 10;
    }

    public final static class DrivebaseConstants {

        // Motor Controllers
        public final static int LEFT_FRONT_CAN_ID = 4;
        public final static int LEFT_REAR_CAN_ID = 5;
        public final static int RIGHT_FRONT_CAN_ID = 6;
        public final static int RIGHT_REAR_CAN_ID = 7;

        // PDP
        public final static int LEFT_FRONT_PDP_ID = 0;
        public final static int LEFT_REAR_PDP_ID = 0;
        public final static int RIGHT_FRONT_PDP_ID = 0;
        public final static int RIGHT_REAR_PDP_ID = 0;

        // DPP
		public final static double LEFT_FRONT_DPR = 1; // meters
		public final static double LEFT_REAR_DPR = 1; // meters
		public final static double RIGHT_FRONT_DPR = 1; // meters
		public final static double RIGHT_REAR_DPR = 1; // meters

        // Current Limits
        //
        public final static int CURRENT_LIMIT_STALL = 40;
        public final static int CURRENT_LIMIT_FREE = 30;
        public final static int SECONDARY_CURRENT_LIMIT = 50; 

        // Misc
        //
        public final static int RAMP_RATE_SECONDS = 1;
        public final static IdleMode DRIVEBASE_IDLE_MODE = IdleMode.kBrake;
    
        //  PID Controller
        //
        public final static double MAX_VEL = 0; // RPM/s
        public final static double MAX_ACCEL = 0; // RPM
        public final static double MIN_VEL = 0; // RPM
        public final static double MAX_ERROR = 0; // Rotations
        public final static double PID_kP = 0;
		public final static double PID_kI = 0;
		public final static double PID_kD = 0;
        public final static double PID_kIZ = 0;
		public final static double PID_kFF = 0;
		public final static double MAX_OUTPUT = 1;
		public final static double MIN_OUTPUT = -1;


        // Velocity Control
        //
        public final static boolean APPLY_VELOCITY_SCALAR = true;
        public final static double DRIVE_WHEEL_RADIUS = 0.01; // meters -- Subject to change
        public final static double DRIVE_GEARBOX_RATIO = 100; // Subject to change
    }

}
