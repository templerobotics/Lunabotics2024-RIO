package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

public class Constants {

    public final static class GlobalConstants{
        // Given side of a mechanism 
        
		public static enum RobotSide{
			Left, Right;
		}

        // Position of a given mechanism
		public static enum MechanismPosition{
			Top, Bottom;
		}

		public static boolean ENABLE_TEST_DASHBOARDS = true;
	}

    public final static class ControllerConstants {
        // Controller IDs
		public final static int XBOX_CONTROLLER_DRIVER = 0;
		public final static int XBOX_CONTROLLER_OPERATOR = 1;

		// Deadzones
		public final static double AXIS_DEADZONE = 0.08; 
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
        public final static int LEFT_FRONT_CAN_ID = 9;
        public final static int LEFT_REAR_CAN_ID = 6;
        public final static int RIGHT_FRONT_CAN_ID = 8;
        public final static int RIGHT_REAR_CAN_ID = 7;

        // PDP
        public final static int LEFT_FRONT_PDP_ID = 0;
        public final static int LEFT_REAR_PDP_ID = 0;
        public final static int RIGHT_FRONT_PDP_ID = 0;
        public final static int RIGHT_REAR_PDP_ID = 0;

        // DPR
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
        public final static double DRIVE_GEARBOX_RATIO = 125; // Subject to change

        // Invert Direction
        //
		public final static boolean INVERT_RIGHT = false;
		public final static boolean INVERT_LEFT = true;
    }

    public final static class DiggingConstants {
        // Belt
		public final static int BELT_1_CAN_ID = 10;
		public final static int BELT_1_PDP_ID = 0;
		public final static int BELT_2_CAN_ID = 11;
		public final static int BELT_2_PDP_ID = 0;

		public final static boolean BELT_INVERT = true;
		public final static double BELT_MAX_ACCEL = 5760; // RPM/s
		public final static double BELT_MAX_VEL = 11000; // RPM
		public final static double BELT_MIN_VEL = 10; // RPM
		public final static double BELT_MAX_ERROR = 50; // Rotations
		public final static double BELT_VELOCITY_SCALAR = 1; // 6.65e-5
		public final static double BELT_kP = 0.000026;
		public final static double BELT_kI = 0.000001;
		public final static double BELT_kD = 0.0001;
		public final static double BELT_kIZ = 20;
		public final static double BELT_kFF = 0.000085;
		public final static double BELT_MAX_OUTPUT = 0.9;
		public final static double BELT_MIN_OUTPUT = -0.9;
		public final static int BELT_CURRENT_LIMIT_STALL = 20;
		public final static int BELT_CURRENT_LIMIT_FREE = 10;
		public final static int BELT_SECNDARY_CURRENT_LIMIT = 22;

        // Linear Actuator
        //
        public final static int LINEAR_2_CAN_ID = 4;
        public final static int LINEAR_3_CAN_ID = 12;
        public final static boolean LINEAR_INVERT = true;
        public final static double LINEAR_DEADBAND = .01;
		public final static double LINEAR_MIN_TRAVEL = 1.438; //1.438; // 0.2876; // 1.438
		public final static double LINEAR_MAX_TRAVEL = 3.3; //3.3; //0.55
		public final static double LINEAR_2_ADJUSTMENT = -0.02;
        public final static double DIGGING_LINEAR_kP = 0.1;
		public final static double DIGGING_LINEAR_kI = 0.000002;
		public final static double DIGGING_LINEAR_kD = 0.000005;
		public final static double DIGGING_LINEAR_kIZ = 20;
		public final static double DIGGING_LINEAR_kFF = 0.000080;

        // Leadscrew
        //
        public final static int LEADSCREW_1_CAN_ID = 2;
        public final static int LEADSCREW_2_CAN_ID = 3;
        public final static boolean LEADSCREW_INVERT = true;
        public final static double LEADSCREW_MAX_ACCEL = 11000 * 0.5; // RPM/s
        public final static double LEADSCREW_MAX_VEL = 11000 * 0.5; // RPM
        public final static double LEADSCREW_MIN_VEL = 10; // RPM
        public final static double LEADSCREW_MAX_ERROR = 1; // Rotations
        public final static double LEADSCREW_MAX_TRAVEL = 13000; // Native Units
        public final static double LEADSCREW_MAX_SPEED = 1; // For leadscrews without PIDs
        public final static double LEADSCREW_EXTENDED_POS = 13000; //TODO: Put a real value here
        public final static double LEADSCREW_kP = 0.0000000015;
        public final static double LEADSCREW_kI = 0.000002;
        public final static double LEADSCREW_kD = 0.000005;
        public final static double LEADSCREW_kIZ = 20;
        public final static double LEADSCREW_kFF = 0.000080;
        public final static double LEADSCREW_MAX_OUTPUT = .9;
        public final static double LEADSCREW_MIN_OUTPUT = -.9;
        public final static int LEADSCREW_CURRENT_LIMIT_STALL = 20;
        public final static int LEADSCREW_CURRENT_LIMIT_FREE = 10;
        public final static int LEADSCREW_SECNDARY_CURRENT_LIMIT = 22;
        public final static IdleMode LEADSCREW_IDLE_MODE = IdleMode.kBrake;
    }

    public final static class DumpingConstants {
        public final static int LINEAR_1_CAN_ID = 5;
        public final static boolean LINEAR_INVERT = true;
        public final static double LINEAR_DEADBAND = .01;
		public final static double LINEAR_MIN_TRAVEL = 0.6; //.01;
		public final static double LINEAR_MAX_TRAVEL = 3.3; //0.55; //0.55
		public final static double LINEAR_2_ADJUSTMENT = -0.02;
        public final static double LINEAR_kP = 0.1;
		public final static double LINEAR_kI = 0.000002;
		public final static double LINEAR_kD = 0.000005;
		public final static double LINEAR_kIZ = 20;
		public final static double LINEAR_kFF = 0.000080;
    }

}