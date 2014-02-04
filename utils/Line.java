package pf.utils;

import java.io.Serializable;
import java.awt.geom.Line2D;


public class Line implements Serializable {

	private Point2D start, end;
	
	// precompute intersection expressions
	private double detXY12;
	private double X1minusX2;
	private double Y1minusY2;
	private double minX, minY, maxX, maxY;
	
	public Line() {
		setCoords(0.0, 0.0, 0.0, 0.0);
	}
	
	public Line(double x1, double y1, double x2, double y2) {
		setCoords(x1, y1, x2, y2);
	}
	
	public String toString() {
		return "line(" + start.x + ", " + start.y + ", " + end.x + ", " + end.y + ")";
	}
	
	public void setCoords(double x1, double y1, double x2, double y2) {
		start = new Point2D(x1, y1);
		end = new Point2D(x2, y2);
		
		detXY12 = (x1 * y2 - y1 * x2);
		X1minusX2 = (x1 - x2);
		Y1minusY2 = (y1 - y2);
		minX = Math.min(x1, x2);
		minY = Math.min(y1, y2);
		maxX = Math.max(x1, x2);
		maxY = Math.max(y1, y2);
	}
	
	public double getX1() {
		return start.x;
	}
	public double getX2() {
		return end.x;
	}
	public double getY1() {
		return start.y;
	}
	public double getY2() {
		return end.y;
	}

	
	/**
	 * Intersection of two lines.
	 * 
	 * @param that other line
	 * @return true iff this and that lines intersect.
	 */

	public boolean intersect(Line line) {
		return Line2D.linesIntersect(start.x,start.y,end.x,end.y,line.getX1(),line.getY1(),line.getX2(),line.getY2());
	}
	

	// Used to compute angle between this line and line only when this line and line intersect.
	// Formula taken from http://www.tpub.com/math2/5.htm
	public double angle(Line line) {
		double thisSlope = (end.getY() - start.getY()) / (end.getX() - start.getX());
		double slope = (line.getY2() - line.getY1()) / (line.getX2() - line.getX1());
		return Math.atan((thisSlope - slope) / (1 + thisSlope * slope));
	}


	public static double angle2(double angleOfFirstLine, Line line) {
		double thisSlope = Math.tan(angleOfFirstLine);
		double slope = (line.getY2() - line.getY1()) / (line.getX2() - line.getX1());
		return Math.atan((thisSlope - slope) / (1 + thisSlope * slope));
	}
}
