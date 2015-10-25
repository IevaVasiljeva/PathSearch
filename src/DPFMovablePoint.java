import geometry.IntPoint;

public class DPFMovablePoint {
	
	final DoublePoint location;
	final double heading;
	double leftWSpeed;
	double rightWSpeed;
	
	public DPFMovablePoint(double x, double y, double heading, double leftSpeed, double rightSpeed) {
		location = new DoublePoint(x, y);
		this.heading = heading;
		this.leftWSpeed = leftSpeed;
		this.rightWSpeed = rightSpeed;
	}

}
