package pf.floors;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;

import pf.utils.Line;
import pf.utils.Rectangle;

public class Area implements Serializable {
	
	protected AreaLayerModel mWallsModel;
	
	private final float GRID_SIZE = 2.0f;
	
	public Area() {
		mWallsModel = new AreaLayerModel();
	}
	
	public void optimize() {
		optimize(GRID_SIZE);
	}
	
	public void optimize(float grid) {
		mWallsModel.computeBuckets(grid);
	}
	
	public void setWalls(Collection<Line> walls) {
		for (Line line: walls) {
			mWallsModel.addWall(line);
		}
	}

		
	public AreaLayerModel getWallsModel() {
		return mWallsModel;
	}
	
	public void setDisplayCropBox(Rectangle box) {
		mWallsModel.setDisplayCropBox(box);
	}

	public void setBoundingBox(Rectangle box) {
		mWallsModel.setBoundingBox(box);
	}
	
	protected static Collection<Line> scaleWalls(Collection<Line> otherWalls, float originX, float originY, float scaleX,
			float scaleY) {
		ArrayList<Line> walls = new ArrayList<Line>();
		for (Line wall : otherWalls) {
			double x1 = scaleX * ((double) wall.getX1() + originX);
			double y1 = scaleY * ((double) wall.getY1() + originY);
			double x2 = scaleX * ((double) wall.getX2() + originX);
			double y2 = scaleY * ((double) wall.getY2() + originY);
			walls.add(new Line(x1, y1, x2, y2));
		}
		return walls;
	}
}
