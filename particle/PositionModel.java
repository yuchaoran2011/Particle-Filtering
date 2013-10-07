package pf.particle;

import pf.floors.Area;

public interface PositionModel {
	public void setPosition(double posX, double posY, int intWeight);
	public void setArea(Area area);	
	public double getHeading();
}
