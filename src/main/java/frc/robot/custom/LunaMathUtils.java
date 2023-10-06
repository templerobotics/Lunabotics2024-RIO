package frc.robot.custom;

public class LunaMathUtils {
	/**
	 * Scale a value from an input range to an output range, inclusive
	 * @param value The value to be scaled
	 * @param inMin The minimum value of the input range
	 * @param inMax The maximum value of the input range
	 * @param outMin The minimum value of the resulting range
	 * @param outMax The maximum value of the resulting range
	 * @return The scaled value
	 */
	public static double scaleBetween(double value, double inMin, double inMax, double outMin, double outMax){
		return (outMax - outMin) * (value - inMin) / (inMax - inMin) + outMin;
	}

	/**
	 * Round a number to the specified number of places
	 * @param val The value to be rounded
	 * @param decimal The number of decimal places to round to
	 * @return The rounded value
	 */
	public static double roundToPlace(double val, int decimal){
		return Math.round(val * Math.pow(10, decimal)) / Math.pow(10, decimal);
	}
}
