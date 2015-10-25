import java.awt.Color;
import java.awt.Point;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.List;

import geometry.IntPoint;
import renderables.*;

public class PotentialFieldsRobot {

	protected final RenderablePoint robotPoint; //Default image if no picture is provided
	protected IntPoint coords; //Position of robot
	protected double heading; //Robot's heading in radians
	protected final int radius; //Size of the robot (Our robot is a circle)
	protected final int sensorRange; //Range of sensors
	protected int stepSize; //How far the robot moves each step, in pixels
	protected final int sensorDensity; //Number of 'lines' the robot uses to see
	protected IntPoint goal;
	private int goalRadius;
	private final List<Renderable> obstacles; //All of the obstacles on the map
	protected List<IntPoint> visibleObstacles;
	protected  int sampleSize;
	private final int sampleSizeDefault;

	/**
	 * Set the robot moving towards a goal on the screen with set radius, step size, etc.
	 * @param imagePath The on-disk location of the image to use for the robot, or null for default
	 * @param startingLocation The coordinates of the starting point
	 * @param goalLocation The coordinates of the goal
	 * @param radius The radius of the robot
	 * @param sensorRange How far the robot can 'see'
	 * @param sensorDensity The number of sensor lines the robot can use
	 * @param goalRadius The width of the goal
	 * @param obstacles A list of all the obstacles on the map 
	 * */
	public PotentialFieldsRobot(IntPoint startingLocation, IntPoint goalLocation, int radius, 
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable>obstacles) {
		robotPoint = new RenderablePoint(startingLocation.x, startingLocation.y);
		robotPoint.setProperties(Color.RED, (float)radius*2);

		this.coords = new IntPoint(startingLocation.x, startingLocation.y);
		heading = calculateHeading(goalLocation);
		this.radius = radius;
		this.sensorRange = sensorRange;
		this.sensorDensity = sensorDensity;
		this.stepSize = 10;
		this.sampleSizeDefault = 2*radius;
		this.goal = goalLocation;
		this.goalRadius = goalRadius;
		this.obstacles = obstacles;
	}

	/**
	 * Move the robot 1 step towards the goal (point of least potential resistance)
	 * @return True if the move is successful, false if there are no viable moves. 
	 **/
	public boolean move() {
		IntPoint moveTo = evaluateSamplePoints(); //Pick a sample point to move towards
		if (moveTo == null) return false;
		IntPoint makeMove = evaluateMovePoints(moveTo); //Find the best move point using current sample as goal
		if (makeMove == null) return false;
		double newHeading = calculateHeading(makeMove);
		moveTowards(newHeading); //Make the move
		return true;
	}

	/**
	 * Evaluate all of the robot's potential movement positions & return the best.
	 * @return The most valuable point
	 */
	protected IntPoint evaluateSamplePoints() {
		List<IntPoint>moves = getSamplePoints();
		//If there's no moves that don't go through obstacles, quit
		if(moves.size() == 0) {
			return null;
		}
		//Value of moves is a function of distance from goal & distance from detected objects
		double[]moveValues = new double[moves.size()];
		for(int i=0;i<moves.size();i++)  {
			moveValues[i] = evalMove(moves.get(i), this.goal);
		}
		return moves.get(minIndex(moveValues)); //Return the lowest valued move
	}

	/**
	 * Evaluate all of the robot's potential movement positions & return the best.
	 * @return The most valuable point
	 */
	protected IntPoint evaluateMovePoints(IntPoint goal) {
		List<IntPoint>moves = getMoveablePoints();
		//If there's no moves that don't go through obstacles, quit
		if(moves.size() == 0) {
			return null;
		}
		//Value of moves is a function of distance from goal & distance from detected objects
		double[]moveValues = new double[moves.size()];
		for(int i=0;i<moves.size();i++)  {
			moveValues[i] = evalMove(moves.get(i), goal);
		}
		return moves.get(minIndex(moveValues)); //Return the lowest valued move
	}

	/**
	 * Get the potential field at point p. The lower the value returned, the better the point is as a move.
	 * @param p The point to evaluate
	 * @return The value of the point
	 */
	protected double evalMove(IntPoint p, IntPoint goal) {
		//Get distances to goal & all visible objects
		double goalDist = (distance(p, goal)-radius) / 10; //Everything is divided by 10 because otherwise the numbers get too big
		double[] obsDists = new double[visibleObstacles.size()];
		for(int i=0;i<visibleObstacles.size();i++) {
			//Distance is set to 0 if it's closer than the radius to the obstacle
			obsDists[i] = (distance(p, visibleObstacles.get(i)) - radius) <= 0 ? 0 : (distance(p, visibleObstacles.get(i)) - radius) / 10;
		}
		//Calculate field power - x^2 so value gets small as distance decreases
		double goalField = Math.pow(goalDist, 2);
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
		return 10*goalField + Math.pow(2*radius,2)*4750*obsField / (sensorDensity*sensorRange);

	}

	/**
	 * Get all of the points the robot can move to - the robot moves 10 pixels forward, 
	 * and can turn upto 12 degrees in either direction to simulate continuous movement.
	 */
	public List<IntPoint> getMoveablePoints() {
		List<IntPoint> moveablePoints = new ArrayList<IntPoint>(5);
		double angleBetween = Math.toRadians(3);
		double currentAngle = mod(heading-Math.toRadians(12), 2*Math.PI);
		for(int i=0;i<9;i++) { 
			//Only make this a 'moveable' point if it does not touch an obstacle
			Line2D.Double line = new Line2D.Double();
			IntPoint p2 = getPointTowards(currentAngle, stepSize);
			line.setLine(coords.x, coords.y, p2.x, p2.y);
			//Check if this line intersects an obstacle, and if so, don't add it
			boolean crash = false;
			for(IntPoint p : visibleObstacles) {
				if (distance(p, p2) <= radius) {
					crash = true;
				}
			}
			if(intersects(line) == null && !crash) {
				moveablePoints.add(p2);
			}
			currentAngle = mod((currentAngle+angleBetween), 2*Math.PI);
		}
		return moveablePoints;
	}

	/**
	 * Get a list of all the sample points evenly distributed in a 180-degree arc in front of the robot 
	 **/
	public List<IntPoint> getSamplePoints() {
		List<IntPoint> moveablePoints = new ArrayList<IntPoint>(sensorDensity);
		double angleBetween = Math.PI / (sensorDensity-1);
		double currentAngle = mod(heading-Math.PI/2, 2*Math.PI);
		sampleSize = distanceToClosestObstacle(); //Sample size changes based on closest obstacle
		for(int i=0;i<sensorDensity;i++) {
			//Only make this a 'moveable' point if it does not touch an obstacle
			Line2D.Double line = new Line2D.Double();
			IntPoint p2 = getPointTowards(currentAngle, sampleSize);
			line.setLine(coords.x, coords.y, p2.x, p2.y);
			if(intersects(line) == null) 
				moveablePoints.add(p2);
			currentAngle+=angleBetween;
		}
		return moveablePoints;
	}

	/**
	 * Get all of the points the robot can move to - the number is equal to the robot's sensor density
	 * spread equally in a 180 degree arc in front of the robot. Additionally, calculate if a sensor
	 * hits an obstacle and make a note of the collision point
	 **/
	public List<IntPoint>getSensorablePoints() {
		List<IntPoint> sensorablePoints = new ArrayList<IntPoint>(sensorDensity);
		visibleObstacles = new ArrayList<IntPoint>();
		double angleBetween = Math.PI/ (sensorDensity-1);
		double currentAngle = mod(heading-Math.PI/2, 2*Math.PI);
		for(int i=0;i<sensorDensity;i++) {
			int sensorRange = this.sensorRange;
			//Check for intersecting obstacles
			IntPoint edge = getPointTowards(currentAngle, sensorRange);
			Line2D.Double sensorLine = new Line2D.Double(new Point(coords.x, coords.y), new Point(edge.x, edge.y));
			IntPoint intersection = intersects(sensorLine);
			if(intersection != null) {
				sensorRange = (int)distance(intersection, coords);
				visibleObstacles.add(intersection);
			}
			sensorablePoints.add(getPointTowards(currentAngle, sensorRange));
			currentAngle+=angleBetween;
		}
		return sensorablePoints;
	}

	/** 
	 * Get the closest point where this line crosses an obstacle - this varies based on the obstacle type
	 * In general, this is achieved by turning the obstacle into a series of lines and calling 
	 * getIntersectionPoint() on the target line and each of the polygon's lines. Once all intersection 
	 * points are found, the closest to the robot is returned. It is assumed all polygons are convex.
	 */
	protected IntPoint intersects(Line2D.Double line) {
		ArrayList<IntPoint> intersections = new ArrayList<IntPoint>();
		for(Renderable obstacle : obstacles) {
			if (obstacle.getClass() == RenderablePolyline.class) {
				ArrayList<Integer> xs = ((RenderablePolyline)obstacle).xPoints;
				ArrayList<Integer> ys = ((RenderablePolyline)obstacle).yPoints;
				for(int i=0;i<xs.size()-1;i++) {
					Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i), xs.get(i+1), ys.get(i+1));
					IntPoint intersect = getIntersectionPoint(line, obsLine);
					if(intersect != null) intersections.add(intersect);
				}
			} else if (obstacle.getClass() == RenderableRectangle.class) {
				/* Rectangle is treated like a polygon but since because it's a 
				 * different class it has to be handled separately - we've got to construct the
				 * polypoints separately (annoyingly)*/
				ArrayList<Integer>xs = new ArrayList<Integer>();
				ArrayList<Integer>ys = new ArrayList<Integer>();
				xs.add(((RenderableRectangle)obstacle).bottomLeftX);
				xs.add(((RenderableRectangle)obstacle).bottomLeftX);
				xs.add(((RenderableRectangle)obstacle).bottomLeftX + ((RenderableRectangle)obstacle).width);
				xs.add(((RenderableRectangle)obstacle).bottomLeftX + ((RenderableRectangle)obstacle).width);

				ys.add(((RenderableRectangle)obstacle).bottomLeftY);
				ys.add(((RenderableRectangle)obstacle).bottomLeftY + ((RenderableRectangle)obstacle).height);
				ys.add(((RenderableRectangle)obstacle).bottomLeftY + ((RenderableRectangle)obstacle).height); 
				ys.add(((RenderableRectangle)obstacle).bottomLeftY);

				for(int i=0;i<xs.size();i++) {
					Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i), 
							xs.get((i+1) % xs.size()), ys.get((i+1) % ys.size()));
					IntPoint intersect = getIntersectionPoint(line, obsLine);
					if(intersect != null) intersections.add(intersect);
				}

			} else if (obstacle.getClass() == RenderablePolygon.class) {
				ArrayList<Integer> xs = ((RenderablePolygon)obstacle).xPoints;
				ArrayList<Integer> ys = ((RenderablePolygon)obstacle).yPoints;
				for(int i=0;i<xs.size();i++) {
					Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i), 
							xs.get((i+1) % xs.size()), ys.get((i+1) % ys.size()));
					IntPoint intersect = getIntersectionPoint(line, obsLine);
					if(intersect != null) intersections.add(intersect);
				}
			} else if (obstacle.getClass() == RenderableOval.class) {
				//ovals are treated as their bounding polygons (90-sided) and they have to be circles
				ArrayList<Integer>xs = new ArrayList<Integer>();
				ArrayList<Integer>ys = new ArrayList<Integer>();
				RenderableOval roval = (RenderableOval) obstacle; 

				for(int i=0;i<90;i++) {
					int trigPoint = (int) (roval.width/2 * Math.cos(i*Math.PI / 45));
					xs.add(roval.centreX + trigPoint);
				}

				for(int i=0;i<90;i++) {
					int trigPoint = (int) (roval.width/2 * Math.sin(i*Math.PI / 45));
					ys.add(roval.centreY + trigPoint);
				}

				for(int i=0;i<xs.size();i++) {
					Line2D.Double obsLine = new Line2D.Double(xs.get(i), ys.get(i), 
							xs.get((i+1) % xs.size()), ys.get((i+1) % ys.size()));
					IntPoint intersect = getIntersectionPoint(line, obsLine);
					if(intersect != null) intersections.add(intersect);
				}

			}
		}
		return intersections.size() == 0 ? null : lowestDist(intersections);
	}

	/**
	 * Get the closest point to the robot's coords
	 * @param points A list of point
	 * @return The point with the smallest distance from the robot
	 **/
	private IntPoint lowestDist(ArrayList<IntPoint> points) {
		int lowest = 0;
		for(int i=0;i<points.size();i++) {
			if (distance(points.get(i), coords) < distance(points.get(lowest), coords))
				lowest = i;
		}
		return points.get(lowest);
	}

	/**
	 * Have the robot move along a certain heading
	 * @param heading The heading to move along. 
	 **/
	protected void moveTowards(double heading) {
		int length = (int) (stepSize * Math.cos(heading));
		int height = (int) (stepSize * Math.sin(heading));
		coords.x += length;
		coords.y += height;
		robotPoint.x = coords.x;
		robotPoint.y = coords.y;
		this.heading = heading;
	}

	/**
	 * Get the point 'step' pixels along the given heading from the robot's position
	 * @param heading The heading to move along
	 * @param step The distance to travel along that heading
	 **/
	protected IntPoint getPointTowards(double heading, int step) {
		int length = (int) (step * Math.cos(heading));
		int height = (int) (step * Math.sin(heading));
		return new IntPoint(coords.x+length, coords.y+height);
	}

	/**
	 * Find the heading that the robot must move to in order to reach a certain point. If
	 * the angle is greater than 60 degrees, truncate it to 60 degrees,=.
	 * @param end The destination point
	 **/
	protected double calculateHeading(IntPoint end) {
		double grad = Math.abs(((double)end.y - (double)coords.y) 
				/ ((double)end.x - (double)coords.x));
		double angle = Math.atan(grad);

		if(end.x - coords.x < 0) {
			if(end.y - coords.y < 0) {
				angle = Math.PI + angle;
			} else {
				angle = Math.PI - angle;
			}
		} else {
			if(end.y - coords.y < 0) {
				angle = (Math.PI * 2) - angle;
			}
		}

		return angle;
	}

	/**
	 * Get the position of the smallest number in an array of doubles 
	 **/
	protected int minIndex(double[] nums) {
		int minIndex = 0;
		for(int i=1;i<nums.length;i++) {
			if(nums[i] < nums[minIndex]) minIndex = i; 
		}
		return minIndex;
	}

	/**
	 * Get the distance between two points. 
	 **/
	protected static double distance(IntPoint a, IntPoint b) {
		return Math.sqrt(Math.pow((a.x-b.x), 2) + Math.pow((a.y-b.y), 2));
	}

	/**
	 * Check if the robot falls within the goal radius. 
	 **/
	public boolean inGoal() {
		return distance(coords, goal) < goalRadius+radius;
	}

	/**
	 * Calculate the intersection point of two lines, or return null if there is no
	 * intersection.
	 * @param line1 The first line
	 * @param line2 The second line
	 * @return The point of intersection, or null.
	 */
	private static IntPoint getIntersectionPoint(Line2D.Double line1, Line2D.Double line2) {
		if (! line1.intersectsLine(line2) ) return null;
		double px = line1.getX1(),
				py = line1.getY1(),
				rx = line1.getX2()-px,
				ry = line1.getY2()-py;
		double qx = line2.getX1(),
				qy = line2.getY1(),
				sx = line2.getX2()-qx,
				sy = line2.getY2()-qy;

		double det = sx*ry - sy*rx;
		if (det == 0) {
			return null;
		} else {
			double z = (sx*(qy-py)+sy*(px-qx))/det;
			if (z==0 ||  z==1) return null;  // intersection at end point
			return new IntPoint(
					(int)(px+z*rx), (int)(py+z*ry));
		}
	}

	/**
	 * Calculate a % b, but always result in a positive answer - java's default syntax returns a
	 * negative number if the dividend is negative, which is unhelpful when my calculations are
	 * performed between 0 and 2PI, rather than -PI and PI.
	 **/
	protected static double mod(double a, double b) {
		return ((a % b) + b) % b;
	}

	/**
	 * Find the distance from the robot to the closest visible obstacle, or some default if
	 * none are visible 
	 **/
	protected int distanceToClosestObstacle() {
		if(visibleObstacles.size()==0) return sampleSizeDefault;
		int closest = 0;
		for(int i=0;i<visibleObstacles.size();i++) 
			if (distance(coords, visibleObstacles.get(i)) < distance(coords, visibleObstacles.get(closest))) 
				closest = i;
		return (int)Math.round(distance(coords, visibleObstacles.get(closest)));
	}

	public IntPoint getPosition() {
		return coords;
	}



	public int getStepSize() {
		return this.stepSize;
	}

	public void setGoal(IntPoint newGoal) {
		this.goal = newGoal;
	}

	public void setGoalRadius(int rad) {
		this.goalRadius = rad;
	}

}