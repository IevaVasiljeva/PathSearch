import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Line2D;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JLabel;

import renderables.*;
import dataStructures.RRTree;
import easyGui.EasyGui;
import geometry.IntPoint;

public class PotentialFields
{
	private final EasyGui gui;
	
	private final int buttonId;
	private final int circleSId;
	private final int circleLId;
	private final int squareSId;
	private final int squareLId;
	private final int randomLineId;
	private final int clearObsId;
	private final int easyCourseId;
	private final int medCourseId;
	private final int hardCourseId;
	private final int shallowCId;
	private final int medCId;
	private final int hardCId;
	
	private final int startXId;
	private final int startYId;
	private final int goalXId;
	private final int goalYId;
	private final int goalRadiusId;
	private final int wheelDistId;
	private final int maxWSId;
	private final int minWSId;
	private final int robotSpeedId;
	private final int linearId;
	private final int squaredId;
	private final int arcId;
	
	private final int frameLength = 1200;
	private final int frameHeight = 900;

	private final int graphicsHeight = 700;
	
	private final int wheelSize = 10;
	private final double wheelDistance = 1;
	private final double startWheelSpeed = 3;
	private final double maxSpeedChange = 1.5;
	private final double maxSpeed = 3;
	private final double minSpeed = 0.01;
	
	private final int sensDensity = 160;
	private final int sensRange = 120;
	private final int robotRadius = 30;
	private int measureMetric;
	
	
	private ArrayList<Renderable>obstacles;
	
	public PotentialFields() {
		//Set up the GUI, labels, buttons
		gui = new EasyGui(frameLength, frameHeight);
		
		buttonId = gui.addButton(5, 5, "Go Little Robot!", this, "buttonAction");
		gui.addButton(2, 5, "Clear Fields", this, "clearButtonAction");
		
		gui.addLabel(0, 0, "Starting X:");
		startXId = gui.addTextField(0, 1, null);
		gui.addLabel(1, 0, "Starting Y:");
		startYId = gui.addTextField(1, 1, null);
		
		gui.addLabel(0, 2, "Goal X:");
		goalXId = gui.addTextField(0, 3, null);
		gui.addLabel(1, 2, "Goal Y:");
		goalYId = gui.addTextField(1, 3, null);
		
		gui.addLabel(0, 4, "Goal Radius:");
		goalRadiusId = gui.addTextField(0, 5, null);
		
		gui.addLabel(0, 6, "Robot Speed (moves/second):");
		robotSpeedId = gui.addTextField(0, 7, null);
		
		gui.addLabel(0, 8, "Distance betwen the wheels:");
		wheelDistId = gui.addTextField(0, 9, null);
		
		gui.addLabel(2, 8, "Maximum wheel speed:");
		maxWSId = gui.addTextField(2, 9, null);

		gui.addLabel(1, 8, "Minimum wheel speed:");
		minWSId = gui.addTextField(1, 9, null);
		
		gui.addLabel(0,-1,"Leave fields blank for random values!");
		
		//Pre-made courses
		gui.addLabel(3, 0, "Select a pre-made course: ");
		easyCourseId = gui.addButton(3, 1, "Easy", this, "easyCourse");
		medCourseId = gui.addButton(3, 2, "Medium", this, "medCourse");
		hardCourseId = gui.addButton(3, 3, "Hard", this, "hardCourse");
		shallowCId = gui.addButton(3, 4, "Shallow C", this, "shallowCCourse");
		medCId = gui.addButton(3, 5, "Medium C", this, "mediumCCourse");
		hardCId = gui.addButton(3, 6, "Hard C", this, "hardCCourse");
		
		//Custom obstacles
		gui.addLabel(4, 0, "Or add in your own obstacles: ");
		circleSId = gui.addButton(4, 1, "Circle (S)", this, "genCircleS");
		circleLId = gui.addButton(4, 2, "Circle (L)", this, "genCircleL");
		squareSId = gui.addButton(4, 3, "Square (S)", this, "genSquareS");
		squareLId = gui.addButton(4, 4, "Square (L)", this, "genSquareL");
		randomLineId = gui.addButton(4, 5, "Line", this, "genLine");
		clearObsId = gui.addButton(4, 6, "Clear Obstacles", this, "clearObs");
		
		// Metric measurement
		gui.addLabel(5, 0, "Choose a metric to measure the distance by: ");
		linearId = gui.addButton(5, 1, "linear", this, "setToLin");
		squaredId = gui.addButton(5, 2, "suqared", this, "setToSquare");
		arcId = gui.addButton(5, 3, "arc", this, "setToArc");
				
		gui.addButton(5, 10, "Quit", this, "quit");
		
		obstacles = new ArrayList<Renderable>();

	}
	
	public void quit() {
		gui.hide();
		System.exit(0);
	}
	
	public void setToLin() {
		measureMetric = 1;
	}
	
	public void setToSquare() {
		measureMetric = 2;
	}
	
	public void setToArc() {
		measureMetric = 3;
	}
	
	// Sets a course to a shallow C obstacle
	public void shallowCCourse() {
		gui.setTextFieldContent(startXId, "100");
		gui.setTextFieldContent(startYId, "250");
		gui.setTextFieldContent(goalXId, "1000");
		gui.setTextFieldContent(goalYId, "500");
		
		RenderablePolyline l = new RenderablePolyline();
		l.addPoint(400, 450);
		l.addPoint(450, 400);
		l.addPoint(470, 350);
		l.addPoint(480, 300);
		l.addPoint(470, 250);
		l.addPoint(450, 200);

		l.setProperties(Color.DARK_GRAY, 10f);
		obstacles.add(l);
		gui.draw(l);
		gui.update();
		
	}
	
	// Sets a course to a medium C obstacle
	public void mediumCCourse() {
		gui.setTextFieldContent(startXId, "100");
		gui.setTextFieldContent(startYId, "250");
		gui.setTextFieldContent(goalXId, "1000");
		gui.setTextFieldContent(goalYId, "500");
		
		RenderablePolyline l = new RenderablePolyline();
		l.addPoint(300, 450);
		l.addPoint(350, 460);
		l.addPoint(400, 450);
		l.addPoint(450, 400);
		l.addPoint(470, 350);
		l.addPoint(480, 300);
		l.addPoint(470, 250);
		l.addPoint(450, 200);
		l.addPoint(400, 175);
		l.addPoint(375, 170);

		l.setProperties(Color.DARK_GRAY, 10f);
		obstacles.add(l);
		gui.draw(l);
		gui.update();

	}
	
	
	// Sets a course to a hard C obstacle
	public void hardCCourse() {
		gui.setTextFieldContent(startXId, "100");
		gui.setTextFieldContent(startYId, "250");
		gui.setTextFieldContent(goalXId, "1000");
		gui.setTextFieldContent(goalYId, "500");
		
		
		RenderablePolyline l = new RenderablePolyline();
		l.addPoint(240, 400);
		l.addPoint(270, 430);
		l.addPoint(300, 450);
		l.addPoint(350, 460);
		l.addPoint(400, 450);
		l.addPoint(450, 400);
		l.addPoint(470, 350);
		l.addPoint(480, 300);
		l.addPoint(470, 250);
		l.addPoint(450, 200);
		l.addPoint(400, 130);
		l.addPoint(350, 100);
		l.addPoint(310, 80);
		l.addPoint(290, 90);
		l.addPoint(265, 110);
		l.addPoint(250, 130);
		l.addPoint(230, 160);

		l.setProperties(Color.DARK_GRAY, 10f);
		obstacles.add(l);
		gui.draw(l);
		gui.update();

	}
	
	
	/**
	 * Action when 'clear fields' button is pressed - reset all text fields in the gui. 
	 **/
	public void clearButtonAction() {
		gui.setTextFieldContent(startXId, "");
		gui.setTextFieldContent(startYId, "");
		gui.setTextFieldContent(goalXId, "");
		gui.setTextFieldContent(goalYId, "");
		gui.setTextFieldContent(goalRadiusId, "");
		gui.setTextFieldContent(wheelDistId, "");
		gui.setTextFieldContent(maxWSId, "");
		gui.setTextFieldContent(minWSId, "");
		gui.setTextFieldContent(robotSpeedId, "");
	}
	
	/**
	 * Set up the 'easy' premade course
	 * */
	public void easyCourse() { 
		gui.setTextFieldContent(startXId, "100");
		gui.setTextFieldContent(startYId, "100");
		gui.setTextFieldContent(goalXId, "1000");
		gui.setTextFieldContent(goalYId, "500");

		RenderableOval r = new RenderableOval(frameLength/2, graphicsHeight/2, 150, 150);
		r.setProperties(Color.DARK_GRAY, 5f, true);
		obstacles.add(r);
		gui.draw(r);
		gui.update();
	}
	
	/**
	 * Set up the 'medium' premade course
	 * */
	public void medCourse() {
		gui.setTextFieldContent(startXId, "0");
		gui.setTextFieldContent(startYId, "0");
		gui.setTextFieldContent(goalXId, "1400");
		gui.setTextFieldContent(goalYId, "700");
		
		RenderableRectangle r = new RenderableRectangle(150, 150, 150, 150);
		r.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r2 = new RenderableRectangle(400, 400, 150, 150);
		r2.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r3 = new RenderableRectangle(700, 425, 150, 150);
		r3.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r4 = new RenderableRectangle(400, 250, 50, 50);
		r4.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r5 = new RenderableRectangle(600, 100, 150, 150);
		r5.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r6 = new RenderableRectangle(1000, 600, 150, 150);
		r6.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r7 = new RenderableRectangle(900, 250, 50, 50);
		r7.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r8 = new RenderableRectangle(1000, 450, 50, 50);
		r8.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r9 = new RenderableRectangle(1150, 350, 50, 50);
		r9.setProperties(Color.DARK_GRAY, 1f, true, false);
		obstacles.add(r);
		obstacles.add(r2);
		obstacles.add(r3);
		obstacles.add(r4);
		obstacles.add(r5);
		obstacles.add(r6);
		obstacles.add(r7);
		obstacles.add(r8);
		obstacles.add(r9);
		gui.draw(obstacles);
		gui.update();
	}
	
	/**
	 * Set up the 'hard' premade course
	 * */
	public void hardCourse() {
		gui.setTextFieldContent(startXId, "0");
		gui.setTextFieldContent(startYId, "0");
		gui.setTextFieldContent(goalXId, "1500");
		gui.setTextFieldContent(goalYId, "950");
		gui.setTextFieldContent(goalRadiusId, "20");
		
		RenderableRectangle r = new RenderableRectangle(150, 200, 150, 150);
		r.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r2 = new RenderableRectangle(400, 400, 150, 150);
		r2.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r4 = new RenderableRectangle(400, 250, 50, 50);
		r4.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r5 = new RenderableRectangle(600, 100, 150, 150);
		r5.setProperties(Color.DARK_GRAY, 1f, true, false);
		RenderableRectangle r7 = new RenderableRectangle(900, 350, 50, 50);
		r7.setProperties(Color.DARK_GRAY, 1f, true, false);
		obstacles.add(r);
		obstacles.add(r2);
		obstacles.add(r4);
		obstacles.add(r5);
		obstacles.add(r7);
		gui.draw(obstacles);
		gui.update();
		
		IntPoint p0 = new IntPoint(800, 700);
		IntPoint p1 = new IntPoint(800, 900);
		IntPoint p2 = new IntPoint(1300, 800);
		IntPoint p3 = new IntPoint(1400, 300);
		IntPoint p4 = new IntPoint(1200, 350);
		RenderablePolyline line = new RenderablePolyline();
		line.addPoint(p0.x, p0.y);
		line.addPoint(p1.x, p1.y);
		line.addPoint(p2.x, p2.y);
		line.addPoint(p3.x, p3.y);
		line.addPoint(p4.x, p4.y);
		line.setProperties(Color.DARK_GRAY, 2f);
		obstacles.add(line);
		gui.draw(line);
		gui.update();
	}
	
	/*Methods to generate random obstacles - circles, squares and lines*/
	
	public void genCircleS() {
		IntPoint centre = randomPoint(frameLength, frameHeight);
		RenderableOval o = new RenderableOval(centre.x, centre.y, 50, 50);
		o.setProperties(Color.MAGENTA, 1f, true);
		obstacles.add(o);
		gui.draw(o);
		gui.update();
	}
	
	public void genCircleL() {
		IntPoint centre = randomPoint(frameLength, frameHeight);
		RenderableOval o = new RenderableOval(centre.x, centre.y, 150, 150);
		o.setProperties(Color.MAGENTA, 1f, true);
		obstacles.add(o);
		gui.draw(o);
		gui.update();
	}
	
	public void genSquareS() {
		IntPoint origin = randomPoint(frameLength-50, frameHeight-5);
		RenderableRectangle r = new RenderableRectangle(origin.x, origin.y, 50, 50);
		r.setProperties(Color.CYAN, 1f, true, false);
		obstacles.add(r);
		gui.draw(r);
		gui.update();
	}
	
	public void genSquareL() {
		IntPoint origin = randomPoint(frameLength-150, frameHeight-150);
		RenderableRectangle r = new RenderableRectangle(origin.x, origin.y, 150, 150);
		r.setProperties(Color.CYAN, 1f, true, false);
		obstacles.add(r);
		gui.draw(r);
		gui.update();
	}
	
	public void genLine() {
		IntPoint p1 = randomPoint(frameLength-200, frameHeight-200);
		IntPoint p2 = randomPoint(frameLength-200, frameHeight-200);
		RenderablePolyline p = new RenderablePolyline();
		p.addPoint(p1.x, p1.y);
		p.addPoint(p2.x, p2.y);
		p.setProperties(Color.ORANGE, 2f);
		obstacles.add(p);
		gui.draw(p);
		gui.update();
		
		//RenderableOval ov = new RenderableOval(centreX, centreY, width, height, startAngle, angularExtent)

	}
	
	/**
	 * Clear all obstacles from the screen 
	 **/
	public void clearObs() {
		gui.unDraw(obstacles);
		obstacles = new ArrayList<Renderable>();
		gui.update();
	}
	
	/**
	 * Start the GUI 
	 **/
	public void runRobot()
	{
		gui.show();
	}
	
	/**
	 * Get the parameters from the text fields and use these to set the robot moving 
	 **/
	public void buttonAction() throws InterruptedException {
		int startX, startY, goalX, goalY, radius, wheelDist, robotSpeed;
		double wheelSpeedMin, wheelSpeedMax;
		Random rand = new Random();
		String startXs = gui.getTextFieldContent(startXId);
		String startYs = gui.getTextFieldContent(startYId);
		String goalXs = gui.getTextFieldContent(goalXId);
		String goalYs = gui.getTextFieldContent(goalYId);
		String radiuss = gui.getTextFieldContent(goalRadiusId);
		String wheelDistance = gui.getTextFieldContent(wheelDistId);
		String minWSpeeds = gui.getTextFieldContent(minWSId);
		String maxWSpeeds = gui.getTextFieldContent(maxWSId);
		String robotSpeeds = gui.getTextFieldContent(robotSpeedId);

		if(startXs.equals("")) startX = rand.nextInt(frameLength); 
		else startX = Integer.parseInt(startXs);
		gui.setTextFieldContent(startXId, ""+startX);
		
		if(startYs.equals("")) startY = rand.nextInt(frameHeight);
		else startY = Integer.parseInt(startYs);
		gui.setTextFieldContent(startYId, ""+startY);
		
		if(goalXs.equals("")) goalX = rand.nextInt(frameLength);
		else goalX = Integer.parseInt(goalXs);
		gui.setTextFieldContent(goalXId, ""+goalX);
		
		if(goalYs.equals("")) goalY = rand.nextInt(frameHeight); 
		else goalY = Integer.parseInt(goalYs);
		gui.setTextFieldContent(goalYId, ""+goalY);
		
		if(radiuss.equals("")) radius = rand.nextInt(70)+30; //Radius is between 30 and 100
		else radius = Integer.parseInt(radiuss);
		gui.setTextFieldContent(goalRadiusId, ""+radius);
	
		if(wheelDistance.equals("")) wheelDist = 1; //Wheel distance default is 1, that's when it works the best
		else wheelDist = Integer.parseInt(wheelDistance);
		gui.setTextFieldContent(wheelDistId, ""+wheelDist);
		
		if(minWSpeeds.equals("")) wheelSpeedMin = 1.5; //between 5 and 180 
		else wheelSpeedMin = Double.parseDouble(minWSpeeds);
		gui.setTextFieldContent(minWSId, ""+wheelSpeedMin);
		
		if(maxWSpeeds.equals("")) wheelSpeedMax = wheelSpeedMin+1.5; //between 1 and 3 
		else wheelSpeedMax = Double.parseDouble(maxWSpeeds);
		gui.setTextFieldContent(maxWSId, ""+wheelSpeedMax);
		
		if(robotSpeeds.equals("")) robotSpeed = 40; //Default speed is 40 moves per second
		else robotSpeed = Integer.parseInt(robotSpeeds);
		gui.setTextFieldContent(robotSpeedId, ""+robotSpeed);
		
		goLittleRobot(new IntPoint(startX, startY), new IntPoint(goalX, goalY), radius, robotSpeed, wheelDist, wheelSpeedMax, wheelSpeedMin);
	}
	
	/**
	 * Set the robot moving towards a goal on the screen with set radius, step size, etc.
	 * @param start The coordinates of the starting point
	 * @param goal The coordinates of the goal
	 * @param goalRad The radius of the goal - if the robot falls within this, it wins
	 * @param robotRadius The width of the robot
	 * @param robotSensorRange How far the robot can 'see'
	 * @param robotSensorDensity The number of sensor lines the robot can use
	 * @param robotSpeed The number of moves per second
	 * */
	public void goLittleRobot(IntPoint start, IntPoint goal, int goalRad, int robotSpeed, int wheelDistance, double maxWheelSpeed, double minSpeedRob) throws InterruptedException
	{
		//Disable all buttons while robot is active
		gui.setButtonEnabled(buttonId, false);
		gui.setButtonEnabled(circleSId, false);
		gui.setButtonEnabled(circleLId, false);
		gui.setButtonEnabled(squareSId, false);
		gui.setButtonEnabled(squareLId, false);
		gui.setButtonEnabled(randomLineId, false);
		gui.setButtonEnabled(clearObsId, false);
		gui.setButtonEnabled(easyCourseId, false);
		gui.setButtonEnabled(medCourseId, false);
		gui.setButtonEnabled(hardCourseId, false);
		gui.setButtonEnabled(linearId, false);
		gui.setButtonEnabled(squaredId, false);
		gui.setButtonEnabled(arcId, false);
		gui.setButtonEnabled(shallowCId, false);
		gui.setButtonEnabled(medCId, false);
		gui.setButtonEnabled(hardCId, false);
		
		//Create the robot, start & end points, renderables
		DynamicPFRobot rob = new DynamicPFRobot(start, goal, robotRadius, sensRange, 
							  sensDensity, goalRad, obstacles, wheelDistance, maxWheelSpeed, minSpeedRob, wheelSize, measureMetric);
		
		RRTree startAndGoal = new RRTree(Color.black);
		startAndGoal.setStartAndGoal(start, goal, goalRad);
		RenderableString rs = null;
		RenderableString rs2 = null;
		RenderablePolyline path = new RenderablePolyline();
		path.setProperties(Color.BLACK, 1f);
		path.addPoint(start.x, start.y);
		
		//Draw the initial set up
		gui.clearGraphicsPanel();
		gui.draw(startAndGoal);
		gui.draw(path);
		gui.draw(obstacles);
		drawRobot(rob);
		gui.update();
		
		int l=0;
		//Loop until the robot reaches the goal or gets stuck
		while (!rob.inGoal()) {
			Thread.sleep(1000/robotSpeed);
			boolean move = rob.move(); //Move 1 step
			
			//If move==false then rob has crashed
			if(move == false) {
				//Draw message to let the user know that Rob is stuck
				RenderableString rs3 = new RenderableString(500, 500, "I'm Stuck :(");
				rs3.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
				rs = new RenderableString(500, 500, "I'm Stuck :(");
				rs.setLayer(456);
				rs.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 32));
				gui.draw(rs);	
				gui.update(); 
				break; //Stop if no actions are available
			}
			
			//Draw the path from start to Rob's position
			path.addPoint(rob.getPosition().x, rob.getPosition().y);
			gui.clearGraphicsPanel();
			gui.draw(startAndGoal);
			gui.draw(path);
			gui.draw(obstacles);
			drawRobot(rob);
			
			//Draw the current path length
			gui.unDraw(rs);
			l += rob.getStepSize();
			rs = new RenderableString(820, 20, "Distance Travelled (pixels): " + l);
			rs.setLayer(456);
			rs.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
			gui.draw(rs);
			
			//Draw the current goal path smoothness
			gui.unDraw(rs2);
			l += rob.getStepSize();
			rs2 = new RenderableString(820, 0, "Path Smoothness Rating: " + (calculateSmoothness(path)));
			rs2.setLayer(456);
			rs2.setProperties(Color.BLUE, new Font(Font.SERIF, Font.BOLD, 14));
			gui.draw(rs2);
			
			gui.update();
		}
		
		//Print metrics to console
		System.out.println("Distance Travelled (pixels): " + l);
		System.out.println("Path Smoothness: " + calculateSmoothness(path));
		
		//Re-enable buttons when finished
		gui.setButtonEnabled(buttonId, true);
		gui.setButtonEnabled(circleSId, true);
		gui.setButtonEnabled(circleLId, true);
		gui.setButtonEnabled(squareSId, true);
		gui.setButtonEnabled(squareLId, true);
		gui.setButtonEnabled(randomLineId, true);
		gui.setButtonEnabled(clearObsId, true);
		gui.setButtonEnabled(easyCourseId, true);
		gui.setButtonEnabled(medCourseId, true);
		gui.setButtonEnabled(hardCourseId, true);
		gui.setButtonEnabled(linearId, true);
		gui.setButtonEnabled(squaredId, true);
		gui.setButtonEnabled(arcId, true);
		gui.setButtonEnabled(shallowCId, true);
		gui.setButtonEnabled(medCId, true);
		gui.setButtonEnabled(hardCId, true);
	}
	
	/**
	 * Draw the robot, it's sensors (in green), and all of the points it can move to (in blue)
	 * */
	private void drawRobot(PotentialFieldsRobot rob)  {
		gui.draw(rob.robotPoint);
		
		gui.draw(((DynamicPFRobot)rob).path);

		for(IntPoint p :rob.getSensorablePoints()) {
			RenderablePolyline r = new RenderablePolyline();
			r.addPoint(rob.getPosition().x, rob.getPosition().y);
			r.addPoint(p.x, p.y);
			r.setProperties(Color.GREEN, 1f);
			RenderablePoint pp = new RenderablePoint(p.x, p.y);
			pp.setProperties(Color.GREEN, 5f);
			gui.draw(r);
			gui.draw(pp);
		}
				
		for(DPFMovablePoint p :((DynamicPFRobot)rob).getDiffDrivePoints()) {
			RenderablePolyline r = new RenderablePolyline();
			r.addPoint(rob.getPosition().x, rob.getPosition().y);
			int visibilityIndex = 18;
			r.addPoint(rob.getPosition().x+(int)Math.round(visibilityIndex*p.location.x), rob.getPosition().y + (int)Math.round(visibilityIndex*p.location.y));
			r.setProperties(Color.BLUE, 3f);
			RenderablePoint pp = new RenderablePoint(rob.getPosition().x+(int)Math.round(visibilityIndex*p.location.x), rob.getPosition().y + (int)Math.round(visibilityIndex*p.location.y));
			pp.setProperties(Color.BLUE, 6f);
//			gui.draw(r);
			gui.draw(pp);
		}
	}
	
	/**
	 * Generate a random point in 2D space in the range ([0-maxX,], [0-maxY]) for
	 * obstacle creation
	 * */
	private static IntPoint randomPoint(int maxX, int maxY) {
		Random rand = new Random();
		IntPoint point = new IntPoint();
		point.x = rand.nextInt(maxX+1);
		point.y = rand.nextInt(maxY+1);
		return point;
	}
	
	/**
	 * Smoothness metric. 0 = completely smooth, high values = not very smooth 
	 **/
	private double calculateSmoothness(RenderablePolyline line) {
		if (line.xPoints.size() < 20) return 0;
		double totalDiff = 0;
		for(int i=0;i<line.xPoints.size()-20;i+=10) {
			IntPoint p1 = new IntPoint(line.xPoints.get(i), line.yPoints.get(i));
			IntPoint pmid = new IntPoint(line.xPoints.get(i+10), line.yPoints.get(i+10));
			IntPoint p2 = new IntPoint(line.xPoints.get(i+20), line.yPoints.get(i+20));
			Line2D pline = new Line2D.Double();
			pline.setLine(p1.x, p1.y, p2.x, p2.y);
			
			double distance = Math.abs((p2.y-p1.y)*pmid.x - (p2.x-p1.x)*pmid.y + p2.x*p1.y - p2.y*p1.x)
							/ Math.sqrt(Math.pow(p2.y-p1.y, 2)+Math.pow(p2.x-p1.x, 2));
			totalDiff += distance;
		}
		return totalDiff / line.xPoints.size();
	}
	
}