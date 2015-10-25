
// A class to represent a point that a differential drive model can lead to 
public class DPFMovablePoint {
	
	// Coordinates of the point (stored in doubles for increased precision)
	final DoublePoint location;
	// The heading that the robot will take if this location is chosen
	final double heading;
	// The wheelspeeds that are required to result in this location
	double leftWSpeed;
	double rightWSpeed;
	
	public DPFMovablePoint(double x, double y, double heading, double leftSpeed, double rightSpeed) {
		location = new DoublePoint(x, y);
		this.heading = heading;
		this.leftWSpeed = leftSpeed;
		this.rightWSpeed = rightSpeed;
	}

}
