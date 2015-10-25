import java.awt.geom.Line2D;
import java.text.spi.DateFormatProvider;
import java.util.ArrayList;
import java.util.List;
import java.util.WeakHashMap;

import geometry.IntPoint;
import renderables.Renderable;

public class DynamicPFRobot extends PotentialFieldsRobot {


	private final double wheelSize;
	private final double wheelDistance;
	private double leftWheelSpeed;
	private double rightWheelSpeed;
	private final double maxSpeedChange;
	private final double maxSpeed;
	private final double minSpeed;
	private final int numberOfSamples = 7;

	//private double heading;

	public DynamicPFRobot(IntPoint startingLocation, IntPoint goalLocation, int radius,
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles, double wheelSize, 
			double wheelDistance, double startingSpeeds, double maxSpeedChange, double maxSpeed, double minSpeed) {
		super(startingLocation, goalLocation, radius, sensorRange, sensorDensity, goalRadius, obstacles);
		this.wheelSize = wheelSize;
		this.wheelDistance = wheelDistance;
		this.leftWheelSpeed = startingSpeeds;
		this.rightWheelSpeed = startingSpeeds;
		this.maxSpeedChange = maxSpeedChange;
		this.maxSpeed = maxSpeed;
		this.minSpeed = minSpeed;
		heading = 5;
	}	

	// Chooses a point from its sample range, & then chooses a diff drive point nearest to it
	public boolean move() {

		System.out.println("Facing: " + Math.toDegrees(heading));

		//Choose a subgoal
		IntPoint moveTo = evaluateSamplePoints();

		System.out.println("Current loc: " + coords.x + " " + coords.y);
		System.out.println("S point result: " + moveTo.x + " " + moveTo.y);

		// Choose a point accessible by differential drive that is closest to the subgoal
		DPFMovablePoint chosenMove = evaluateDiffSamplePoints(moveTo);

		moveTowards(chosenMove); //Make the move
		return true;
	}

	// Makes a move towards the chosen point - updates the necessary parameters
	protected void moveTowards(DPFMovablePoint newPoint) {

		System.out.println("Move x diff: " + newPoint.location.x);
		System.out.println("Move y diff: " + newPoint.location.y);

		// Update the robot location with values rounded to the nearest int
		coords.x += Math.round(newPoint.location.x);
		coords.y += Math.round(newPoint.location.y);
		robotPoint.x = coords.x;
		robotPoint.y = coords.y;

		// Update whell speeds
		this.rightWheelSpeed = newPoint.rightWSpeed;
		this.leftWheelSpeed = newPoint.leftWSpeed;

		System.out.println("RW: " + rightWheelSpeed);
		System.out.println("LW: " + leftWheelSpeed);

		// Update the heading
		this.heading = newPoint.heading;
	}

	// Calculates the location and heading for the point where the robot will end up if the given wheel speeds are used.
	// Formulae from the source at http://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf (accessed on 24.10.2015) are used,
	// all credit goes to Dudek and Jenkin (Computational Principles of Mobile Robotics)
	private DPFMovablePoint getPointTowards(double rightWSpeed, double leftWSpeed) {	

		// The rate of rotation around the ICC (instantaneous centre of curvature), calculated in degrees and parsed to radians for further operations
		double rateOfRotation = wheelSize*(rightWSpeed-leftWSpeed)/wheelDistance;
		double rateOfRotRadians = mod(Math.toRadians(rateOfRotation), 2*Math.PI);

		// Distance to the ICC
		double distToICC = (rightWSpeed+leftWSpeed)/(2*(rightWSpeed-leftWSpeed));

		// If wheel speeds are the same, return, as otherwise the result for change in X and Y is zero
		if (Math.floor(rateOfRotation*100)==0 || Math.ceil(rateOfRotation*100)==0) {
			return null;
		}

		// Calculate the changes in the position of the robot and its new heading
		double newX = distToICC*(Math.sin(heading)*Math.cos(rateOfRotRadians)+Math.sin(rateOfRotRadians)*Math.cos(heading) - Math.sin(heading));
		double newY = distToICC*(Math.sin(heading)*Math.sin(rateOfRotRadians)-Math.cos(rateOfRotRadians)*Math.cos(heading) + Math.cos(heading));
		double newAngle = mod(heading + rateOfRotRadians, 2*Math.PI);

		// If the changes are too small, increase them up till they are at least > 0
		while (newX != 0 && Math.abs(newX) < 1  && newY != 0 && Math.abs(newY) < 1) {
			newX*=10;
			newY*=10;
		}

		// If the results are too big, decrease them till they are within ones
		while (Math.abs(newX) >10 && Math.abs(newY) >10) {
			newX/=10;
			newY/=10;
		}

		// Return the results in the form of DPFMovablePoint, with wheel speeds added later (thus they are passed as -1 for now)
		DPFMovablePoint results = new DPFMovablePoint(newX, newY, newAngle, -1, -1);
		return results;
	}


	// Returns a list of points accessible using differential drive mechanics
	public List<DPFMovablePoint> getDiffDrivePoints() {

		List<DPFMovablePoint> moveablePoints = new ArrayList<DPFMovablePoint>();

		// Can change the speeds (and thus rotate) only within a particular range
		double speedChangeOption = maxSpeedChange/numberOfSamples;

		// Go through the possible speed changes and store the locations they would lead to
		for (int i=0; i<=numberOfSamples; i++) {
			double rightSpeed = rightWheelSpeed + speedChangeOption*i - maxSpeedChange/2;

			if (rightSpeed<minSpeed) {
				continue;
			}

			if (rightSpeed>maxSpeed) {
				break;
			}

			for (int j=0; j<=numberOfSamples; j++) {
				double leftSpeed = leftWheelSpeed + speedChangeOption*j - maxSpeedChange/2;

				if (leftSpeed<minSpeed) {
					continue;
				}

				if (leftSpeed>maxSpeed || (i+j)*speedChangeOption/2>maxSpeedChange) {
					break;
				}

				DPFMovablePoint p2 = getPointTowards(rightSpeed, leftSpeed);

				if (p2==null) {
					continue;
				}
				p2.rightWSpeed = rightSpeed;
				p2.leftWSpeed = leftSpeed;
				moveablePoints.add(p2);
			}
		}

		//if (moveablePoints.isEmpty())

		//TODO check for crash

		return moveablePoints;
	}


	private DPFMovablePoint evaluateDiffSamplePoints( IntPoint goal) {

		List<DPFMovablePoint>moves = getDiffDrivePoints();

		//Value of moves is a function of distance from goal & distance from detected objects
		double[] moveValues = new double[moves.size()];

		DPFMovablePoint bestPoint = moves.get(0);
		//		double bestScore = evaluate(bestPoint.location, goal);
		double bestScore = getArcGoalEstimate(bestPoint, goal);

		for(DPFMovablePoint currentPoint: moves) {
			//			double currentScore = evaluate(currentPoint.location, goal);

			//			DoublePoint p2 = new DoublePoint(6, -2);
			//			IntPoint p3 = new IntPoint(5,5);
			//			IntPoint p1 = new IntPoint(2, -4);
			//			
			//			getArcGoalEstimate(p2, p3, p1);

			double currentScore = getArcGoalEstimate(currentPoint, goal);

			if (currentScore < bestScore) {
				bestPoint = currentPoint;
				bestScore = currentScore;
			}
		}

		return bestPoint; //Return the lowest valued move

	}



	//Estimates the distance to the goal using Euclidean straight line
	private double getLinearGoalEstimate(DoublePoint point, IntPoint goal) {
		DoublePoint move = new DoublePoint(coords.x+point.x, coords.y+point.y);
		double goalDist = (distance(move, goal));
		return goalDist;
	}

	//Estimates the distance to the goal using Euclidean squared distance
	private double getSquareGoalEsitamte(DoublePoint point, IntPoint goal) {
		DoublePoint move = new DoublePoint(coords.x+point.x, coords.y+point.y);
		double goalDist = Math.pow((distance(move, goal)), 2);
		return goalDist;
	}

	protected static double distance(DoublePoint a, IntPoint b) {
		return Math.sqrt(Math.pow((a.x-b.x), 2) + Math.pow((a.y-b.y), 2));
	}

	//Estimates the distance to the goal using Euclidean squared distance
	// TODO Take the heading of the double point?
	private double getArcGoalEstimate(DPFMovablePoint point, IntPoint goal) {
		DoublePoint pointLoc = new DoublePoint(coords.x + point.location.x, coords.y + point.location.y);

		double nextByHeadingX = pointLoc.x + Math.cos(point.heading)*15;
		double nextByHeadingY = pointLoc.y + Math.sin(point.heading)*15;

		//		double firstSlope = (goal.y - pointLoc.y)/(goal.x - pointLoc.x);
		//		double secondSlope = (pointLoc.y - coords.y)/(pointLoc.x - coords.x);
		//		
		//		double centreX = (firstSlope*secondSlope*(coords.y-goal.y) + firstSlope*(pointLoc.x+coords.x)-secondSlope*(goal.x+pointLoc.x))/(2*(firstSlope-secondSlope));
		//		double centreY = -(1/secondSlope)*(centreX-(pointLoc.x + coords.x)/2) + (pointLoc.y + coords.y)/2;
		//		
		//		double radius = Math.sqrt(Math.pow(pointLoc.x - centreX, 2) + Math.pow(pointLoc.y - centreY, 2));
		//		
		//		double slopeCurrCentre = (goal.y - centreY)/(goal.x - centreX);
		//		double slopeGoalCentre = (coords.y - centreY)/(coords.x - centreX);
		//		double angleFirstSlope = Math.atan(slopeCurrCentre);
		//		double angleSecondSlope = Math.atan(slopeGoalCentre);


		double firstSlope = (goal.y - nextByHeadingY)/(goal.x - nextByHeadingX);
		double secondSlope = (nextByHeadingY - pointLoc.y)/(nextByHeadingX - pointLoc.x);

		double centreX = (firstSlope*secondSlope*(pointLoc.y-goal.y) + firstSlope*(nextByHeadingX+pointLoc.x)-secondSlope*(goal.x+nextByHeadingX))/(2*(firstSlope-secondSlope));
		double centreY = -(1/secondSlope)*(centreX-(nextByHeadingX + pointLoc.x)/2) + (nextByHeadingY + pointLoc.y)/2;

		double radius = Math.sqrt(Math.pow(nextByHeadingX - centreX, 2) + Math.pow(nextByHeadingY - centreY, 2));

		double slopeCurrCentre = (goal.y - centreY)/(goal.x - centreX);
		double slopeGoalCentre = (pointLoc.y - centreY)/(pointLoc.x - centreX);
		double angleFirstSlope = Math.atan(slopeCurrCentre);
		double angleSecondSlope = Math.atan(slopeGoalCentre);
		double portionAngle = angleFirstSlope - angleSecondSlope; 


		double portion = Math.abs(portionAngle)*radius;

		//Get distances to goal
		return portion;
	}

}
