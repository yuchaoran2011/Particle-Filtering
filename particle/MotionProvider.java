package pf.particle;

import java.util.ArrayList;
import java.util.List;


public abstract class  MotionProvider {
	
	//protected final static String TAG = "PositionProvider";
	
	protected List<MotionUpdateListener> mListeners;
	
	
	public MotionProvider() {
		mListeners = new ArrayList<MotionUpdateListener>();
	}
	
	public void register(MotionUpdateListener listener) {
		mListeners.add(listener);
	}

	public void unregister(MotionUpdateListener listener) {
		mListeners.remove(listener);
	}

	protected void notifyPositionUpdate(float orientation, float posX,
			float posY) {
		for (MotionUpdateListener listener : mListeners) {
				listener.positionChanged(orientation, posX, posY);
		}
	}

	
	public abstract void resume();
	public abstract void stop();
	public abstract void setPaused(boolean paused);
	public void reset() {}
	
}
