package pf.particle;

import pf.floors.Area;
import pf.particle.MotionProvider;
import pf.particle.MotionUpdateListener;


public interface PositionModel extends MotionUpdateListener {
	//public PositionRenderer getRenderer();
	public void setPositionProvider(MotionProvider provider);
	public void setPosition(double posX, double posY, int intWeight);
	public void setArea(Area area);	
	public double getHeading();
}
