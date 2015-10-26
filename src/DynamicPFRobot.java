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

	// Were used for the previous diff drive sampling method
//	private double leftWheelSpeed;
//	private double rightWheelSpeed;
	
	private final double wheelDistance;
	private final double maxSpeed;
	private final double minSpeed;
	private final int numberOfSamples = 10;
	private final int wheelSize;
	private final int measureMetric;
	private int eveningOut;
	int distanceTraveled;
	
	// Path of estimated movement
	Renderable path;

	// Initialising robot
	public DynamicPFRobot(IntPoint startingLocation, IntPoint goalLocation, int radius,
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles, double wheelDist, double maxSpeed, double minSpeed, int wheelSize, int measureMetric) {
		super(startingLocation, goalLocation, radius, sensorRange, sensorDensity, goalRadius, obstacles);
		this.wheelDistance = wheelDist;
		this.maxSpeed = maxSpeed;
		this.minSpeed = minSpeed;
		this.wheelSize = wheelSize;
		this.measureMetric = measureMetric;
		distanceTraveled = 0;
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
		int newX = coords.x + (int)Math.round(newPoint.location.x);
		int newY = coords.y + (int)Math.round(newPoint.location.y);
		double pathTraversed = distance(coords, new IntPoint(newX, newY));
		coords.x += newX;
		coords.y += newY;
		robotPoint.x = coords.x;
		robotPoint.y = coords.y;

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
		double newX = 10*distToICC*(Math.sin(heading)*Math.cos(rateOfRotRadians)+Math.sin(rateOfRotRadians)*Math.cos(heading) - Math.sin(heading));
		double newY = 10*distToICC*(Math.sin(heading)*Math.sin(rateOfRotRadians)-Math.cos(rateOfRotRadians)*Math.cos(heading) + Math.cos(heading));
		double newAngle = mod(heading + rateOfRotRadians, 2*Math.PI);

		// Return the results in the form of DPFMovablePoint
		DPFMovablePoint results = new DPFMovablePoint(newX, newY, newAngle, rightWSpeed, leftWSpeed);
		return results;
	}


	// Returns a list of points accessible using differential drive mechanics
	public List<DPFMovablePoint> getDiffDrivePoints() {

		// Points stored as DPFMovablePoint because additional information apart from the coordinates (wheel speeds, heading) is necessary
		List<DPFMovablePoint> moveablePoints = new ArrayList<DPFMovablePoint>();

		// Look at all the possible wheel speed combinations
		double rightSpeed = minSpeed;
		double increment = maxSpeed/numberOfSamples;
		
		while (rightSpeed<maxSpeed) {
			rightSpeed += increment;
			double leftSpeed = minSpeed;
			while (leftSpeed<maxSpeed) {
				leftSpeed += increment;
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
		eveningOut = -1;
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
	
	
	private double fracProgLinear(DPFMovablePoint point, IntPoint goal) {

		double fracProg = (1 - (distanceTraveled)/(distanceTraveled+linearDistEstimate(point.location, goal)));
		return getObstaclePot(point.location)*fracProg;
	}
	

	//Estimates the distance to the goal using Euclidean straight line
	private double getLinearGoalEstimate(DPFMovablePoint point, IntPoint goal) {
		
		DoublePoint moveResult = new DoublePoint(coords.x+point.location.x, coords.y+point.location.y);
				
		// Calculate the straight line distance
		double result = linearDistEstimate(moveResult, goal);
		
		// Find a visualisation for the estimated path to the goal
		RenderablePolyline path = new RenderablePolyline();
		path.addPoint(this.goal.x, this.goal.y);
		path.addPoint(coords.x + (int)Math.round(point.location.x), coords.y + (int)Math.round(point.location.y));
		path.setProperties(Color.ORANGE, 1f);
		point.path = path;

		result = evenOutResult(result, moveResult);
		
		return result;
	}
	
	// Calculate straight line distance to the goal
	private double linearDistEstimate(DoublePoint point, IntPoint goal) {
		// Calculate the straight line distance to the subgoal and from the subgoal to the goal
		double goalDist = (distance(point, goal));
		double subGoalToGoal = (distance(goal, this.goal));
		return subGoalToGoal+goalDist;
	}
	
	// Evens out the results so that obstacle repulsion has larger impact than goal attraction
	private double evenOutResult(double goalPot, DoublePoint subGoal) {
		double obstaclePot = getObstaclePot(subGoal);
		if (eveningOut == -1) {
			eveningOut = 0;
			while (obstaclePot!=0 && obstaclePot*100<goalPot) {
				goalPot = goalPot/10;
				eveningOut++;
			}
		}
		else {
			goalPot = goalPot/Math.pow(10, eveningOut);
		}
		return goalPot + obstaclePot;
	}
	

	//Estimates the distance to the goal using Euclidean squared distance
	private double getSquareGoalEsitamte(DPFMovablePoint point, IntPoint goal) {
		DoublePoint moveResult = new DoublePoint(coords.x+point.location.x, coords.y+point.location.y);		
		
		// Calculate the straight line distance squared
		double result = squareDistEstimate(moveResult, goal);
		
		// Find a visualisation for the estimated path to the goal
		RenderablePolyline path = new RenderablePolyline();
		path.addPoint(this.goal.x, this.goal.y);
		path.addPoint(coords.x + (int)Math.round(point.location.x), coords.y + (int)Math.round(point.location.y));
		path.setProperties(Color.ORANGE, 1f);
		point.path = path;
		
		result = evenOutResult(result, moveResult);
		
		return result;
	}
	
	// Calculate the straight line distance squared	
	private double squareDistEstimate(DoublePoint point, IntPoint goal) {
		double goalDist = Math.pow((distance(point, goal)), 2);
		
		double subGoalToGoal = Math.pow((distance(goal, this.goal)), 2);
		
		return subGoalToGoal+goalDist;
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
		
		int startAngle = (int)Math.round(Math.toDegrees(angleFirstSlope));
		while (startAngle < 0) {
			startAngle += 360;
		}
		int endAngle = (int)Math.round(Math.toDegrees(angleSecondSlope));
		while (endAngle < 0) {
			endAngle += 360;
		}
		
		if (startAngle>endAngle) {
			double temp = startAngle;
			startAngle = endAngle;
			endAngle = startAngle;
		}
		
		// Find a visualisation for the estimated path to the goal
		RenderableOval path = new RenderableOval((int)Math.round(centreX), (int)Math.round(centreY), (int)Math.round(radius), (int)Math.round(radius), startAngle, (int)Math.round(Math.toDegrees(portionAngle)));
		path.setProperties(Color.ORANGE, 1f, false);
		point.path = path;
		
		double obstaclePot = getObstaclePot(pointLoc);
		if (eveningOut == -1) {
			eveningOut = 0;
			while (obstaclePot!=0 && obstaclePot*100<portion) {
				portion = portion/10;
				eveningOut++;
			}
		}
		else {
			portion = portion/Math.pow(10, eveningOut);
		}

		//Get distances to goal
		return obstaclePot+portion;
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

		System.out.println("Obst pot: " + Math.pow(2*radius,2)*4750*obsField / (sensorDensity*sensorRange));
		
		return Math.pow(2*radius,2)*4750*obsField / (sensorDensity*sensorRange);
	}

}
