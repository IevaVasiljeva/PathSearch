import java.awt.Color;
import java.awt.geom.Line2D;
import java.text.spi.DateFormatProvider;
import java.util.ArrayList;
import java.util.List;
import java.util.WeakHashMap;

import geometry.IntPoint;
import renderables.Renderable;
import renderables.RenderableOval;
import renderables.RenderablePolyline;

// A class for modelling a robot that would incorporate the mechanics behind potential fields
public class DynamicPFRobot extends PotentialFieldsRobot {

	private final double wheelDistance;
	private double leftWheelSpeed;
	private double rightWheelSpeed;
	private final double maxSpeedChange;
	private final double maxSpeed;
	private final double minSpeed;
	private final int numberOfSamples = 7;
	private final int wheelSize;
	private final int measureMetric;
	
	// Path of estimated movement
	Renderable path;

	// Initialising robot
	public DynamicPFRobot(IntPoint startingLocation, IntPoint goalLocation, int radius,
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles, double wheelDist, double startingSpeeds, double maxSpeedChange, double maxSpeed, double minSpeed, int wheelSize, int measureMetric) {
		super(startingLocation, goalLocation, radius, sensorRange, sensorDensity, goalRadius, obstacles);
		this.wheelDistance = wheelDist;
		this.leftWheelSpeed = startingSpeeds;
		this.rightWheelSpeed = startingSpeeds;
		this.maxSpeedChange = maxSpeedChange;
		this.maxSpeed = maxSpeed;
		this.minSpeed = minSpeed;
		this.wheelSize = wheelSize;
		this.measureMetric = measureMetric;
	}	

	// Chooses a point from its sample range, and then chooses a point reachable by differential drive that is nearest to the chosen sample point
	public boolean move() {

		//Choose a subgoal
		IntPoint moveTo = evaluateSamplePoints();

		// Choose a point accessible by differential drive that is closest to the subgoal
		DPFMovablePoint chosenMove = evaluateDiffSamplePoints(moveTo);

		// Make a move
		moveTowards(chosenMove);
		return true;
	}

	// Makes a move towards the chosen point - updates the necessary parameters
	protected void moveTowards(DPFMovablePoint newPoint) {

		// Set the estimated path image
		this.path = newPoint.path;

		// Update the robot location with values rounded to the nearest int
		coords.x += Math.round(newPoint.location.x);
		coords.y += Math.round(newPoint.location.y);
		robotPoint.x = coords.x;
		robotPoint.y = coords.y;

		// Update wheel speeds
		this.rightWheelSpeed = newPoint.rightWSpeed;
		this.leftWheelSpeed = newPoint.leftWSpeed;

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

		// Return the results in the form of DPFMovablePoint
		DPFMovablePoint results = new DPFMovablePoint(newX, newY, newAngle, rightWSpeed, leftWSpeed);
		return results;
	}


	// Returns a list of points accessible using differential drive mechanics
	public List<DPFMovablePoint> getDiffDrivePoints() {

		// Points stored as DPFMovablePoint because additional information apart from the coordinates (wheel speeds, heading) is necessary
		List<DPFMovablePoint> moveablePoints = new ArrayList<DPFMovablePoint>();

		// Can change the speeds (and thus rotate) only within a particular range
		double speedChangeOption = maxSpeedChange/numberOfSamples;

		// Go through the possible speed changes and store the locations they would lead to
		for (int i=0; i<=numberOfSamples; i++) {
			double rightSpeed = rightWheelSpeed + speedChangeOption*i - maxSpeedChange/2;

			// If speed change smaller than the allowed one, don't look at this option
			if (rightSpeed<minSpeed) {
				continue;
			}

			// If speed change bigger - break, as the rest of them will be even bigger
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
				
				moveablePoints.add(p2);
			}
		}
		return moveablePoints;
	}


	// Evaluates points that are reachable by diff drive in a relation to the chosen subgoal, and returns the one with the lowest potential
	private DPFMovablePoint evaluateDiffSamplePoints(IntPoint goal) {

		// Obtains points that we can get to 
		List<DPFMovablePoint>moves = getDiffDrivePoints();

		// Find the one with the smallest potential by looping through all of them
		DPFMovablePoint bestPoint = moves.get(0);
		
		double bestScore;
		// Different metrics can be chosen for evaluation by the user
		switch (measureMetric) {
		case 1:
			bestScore = getLinearGoalEstimate(bestPoint, goal);
			break;
		case 2:
			bestScore = getSquareGoalEsitamte(bestPoint, goal);
			break;
		case 3:
			bestScore = getArcGoalEstimate(bestPoint, goal);
			break;
		default:
			bestScore = getSquareGoalEsitamte(bestPoint, goal);
			break;
		}
		
		for(DPFMovablePoint currentPoint: moves) {			
			double currentScore = 0;
			switch (measureMetric) {
			case 1:
				currentScore = getLinearGoalEstimate(currentPoint, goal);
				break;
			case 2:
				currentScore = getSquareGoalEsitamte(currentPoint, goal);
				break;
			case 3:
				currentScore = getArcGoalEstimate(currentPoint, goal);
				break;
			default:
				currentScore = getSquareGoalEsitamte(currentPoint, goal);
				break;
			}

			if (currentScore < bestScore) {
				bestPoint = currentPoint;
				bestScore = currentScore;
			}
		}

		// Return the lowest valued move
		return bestPoint;

	}

	//Estimates the distance to the goal using Euclidean straight line
	private double getLinearGoalEstimate(DPFMovablePoint point, IntPoint goal) {
				
		// Calculate the straight line distance
		DoublePoint move = new DoublePoint(coords.x+point.location.x, coords.y+point.location.y);
		double goalDist = (distance(move, goal));
		
		// Find a visualisation for the estimated path to the goal
		RenderablePolyline path = new RenderablePolyline();
		path.addPoint(this.goal.x, this.goal.y);
		path.addPoint(coords.x + (int)Math.round(point.location.x), coords.y + (int)Math.round(point.location.y));
		path.setProperties(Color.ORANGE, 1f);
		point.path = path;
		
		return getObstaclePot(move) + goalDist;
	}

	//Estimates the distance to the goal using Euclidean squared distance
	private double getSquareGoalEsitamte(DPFMovablePoint point, IntPoint goal) {
				
		// Calculate the straight line distance squared
		DoublePoint move = new DoublePoint(coords.x+point.location.x, coords.y+point.location.y);
		double goalDist = Math.pow((distance(move, goal)), 2);
		
		// Find a visualisation for the estimated path to the goal
		RenderablePolyline path = new RenderablePolyline();
		path.addPoint(this.goal.x, this.goal.y);
		path.addPoint(coords.x + (int)Math.round(point.location.x), coords.y + (int)Math.round(point.location.y));
		path.setProperties(Color.ORANGE, 1f);
		point.path = path;
		
		return getObstaclePot(move) + goalDist;
	}

	// Calculates the distance using Pitaghor's theorem
	protected static double distance(DoublePoint a, IntPoint b) {
		return Math.sqrt(Math.pow((a.x-b.x), 2) + Math.pow((a.y-b.y), 2));
	}

	//Estimates the distance by constructing an arch using three points - goal, chosen point and a point where the chosen point is heading towards
	private double getArcGoalEstimate(DPFMovablePoint point, IntPoint goal) {
				
		goal = this.goal;
		
		DoublePoint pointLoc = new DoublePoint(coords.x + point.location.x, coords.y + point.location.y);

		double nextByHeadingX = pointLoc.x + Math.cos(point.heading)*2;
		double nextByHeadingY = pointLoc.y + Math.sin(point.heading)*2;
		
		double firstSlope = (goal.y - nextByHeadingY)/(goal.x - nextByHeadingX);
		double secondSlope = (nextByHeadingY - pointLoc.y)/(nextByHeadingX - pointLoc.x);

		double centreX = (firstSlope*secondSlope*(pointLoc.y-goal.y) + firstSlope*(nextByHeadingX+pointLoc.x)-secondSlope*(goal.x+nextByHeadingX))/(2*(firstSlope-secondSlope));
		double centreY = -(1/secondSlope)*(centreX-(nextByHeadingX + pointLoc.x)/2) + (nextByHeadingY + pointLoc.y)/2;

		double radius = 2*Math.sqrt(Math.pow(nextByHeadingX - centreX, 2) + Math.pow(nextByHeadingY - centreY, 2));

		double slopeGoalCentre = (goal.y - centreY)/(goal.x - centreX);
		double slopeCurrCentre = (pointLoc.y - centreY)/(pointLoc.x - centreX);
		double angleFirstSlope = Math.atan(slopeCurrCentre);
		double angleSecondSlope = Math.atan(slopeGoalCentre);
		double portionAngle = Math.abs(angleFirstSlope - angleSecondSlope); 


		double portion = Math.abs(portionAngle)*radius;
		
		//TODO figure out the angles if I can
		int startAngle = (int)Math.round(Math.toDegrees(angleFirstSlope));
		if (pointLoc.y-centreY < 0) {
			startAngle += 180;
		}
		int endAngle = (int)Math.round(Math.toDegrees(angleSecondSlope));
		if (goal.y-centreY < 0) {
			startAngle += 180;
		}
		
		if (startAngle<endAngle) {
			int temp = startAngle;
			startAngle = endAngle;
			endAngle = temp;
		}
		
		// Find a visualisation for the estimated path to the goal
		RenderableOval path = new RenderableOval((int)Math.round(centreX), (int)Math.round(centreY), (int)Math.round(radius), (int)Math.round(radius));
		
//		RenderableOval path = new RenderableOval((int)Math.round(centreX), (int)Math.round(centreY), (int)Math.round(radius), (int)Math.round(radius));

		
		path.setProperties(Color.ORANGE, 1f, false);
		point.path = path;

		//Get distances to goal
		return getObstaclePot(pointLoc)+portion;
	}
	
	
	protected double getObstaclePot(DoublePoint p) {
		//Get distances to goal & all visible objects
		double[] obsDists = new double[visibleObstacles.size()];
		for(int i=0;i<visibleObstacles.size();i++) {
			//Distance is set to 0 if it's closer than the radius to the obstacle
			obsDists[i] = (distance(p, visibleObstacles.get(i)) - radius) <= 0 ? 0 : (distance(p, visibleObstacles.get(i)) - radius) / 10;
		}

		//obs. field power is sum of all obstacles, and gets v. large as distance decreases and vice versa
		double obsField = 0;
		for(int i=0;i<visibleObstacles.size();i++) {
			if(obsDists[i] <= 0) {
				obsField = Double.MAX_VALUE;
				break;
			} else if (obsDists[i] > sensorRange) {
				continue;
			}
			obsField += Math.pow(Math.E, -1 / ((sensorRange) - obsDists[i])) / (obsDists[i]);
		}
		return Math.pow(2*radius,2)*4750*obsField / (sensorDensity*sensorRange);
	}

}
