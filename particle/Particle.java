package pf.particle;

import java.util.Arrays;

import pf.distr.NormalDistribution;


public class Particle implements Cloneable {

	double[] state; // x, y, heading, stride length, intWeight
	double x, y;
	double hdg, stepLength, doubleWeight;
	int areaIndex, intWeight; 

	/*
	public Particle(double posX, double posY, double heading, double stepLength) {
		this.state = new double[] { posX, posY, heading, stepLength };
		this.areaIndex = 0;
	}
	*/
	
	public Particle(double posX, double posY, double heading, double stepLength, int areaIndex, int intWeight) {
		state = new double[] { posX, posY, heading, stepLength, intWeight };
		x = posX;
		y = posY;
		hdg = heading;
		this.stepLength = stepLength; 
		this.areaIndex = areaIndex;
		this.intWeight = intWeight;
		this.doubleWeight = 0f;
	}


	// Constructor for a dead particle
	public Particle() {
		this(0f, 0f, 0f, 0f, 0, 0);
	}



	// polar version
	public static Particle polarNormalDistr(double meanX, double meanY,
			double sigma, double headingDeflection, double headingSpread,
			double stepLength, double stepSpread, int areaIdx, int intWeight) {

		double angle = 2 * Math.PI * Math.random();
		double distance = sigma * NormalDistribution.inverse(Math.random());
		
		double x = meanX + (distance * Math.cos(angle));
		double y = meanY + (distance * Math.sin(angle));
		
		double randomHeading = headingDeflection + headingSpread * NormalDistribution.inverse(Math.random());
		double randomStepLength = stepLength + stepSpread * NormalDistribution.inverse(Math.random());
		return new Particle(x, y, randomHeading, randomStepLength, areaIdx, intWeight);
	}



	public static Particle evenSpread(double meanX, double meanY, double sizeX, double sizeY,
			double headingDeflection, double headingSpread,
			double stepLength, double stepSpread, int areaIdx, int intWeight) {
		
		
		double x = meanX + ((Math.random() - 0.5) *  sizeX);
		double y = meanY + ((Math.random() - 0.5) * sizeY);
		
		double randomHeading = headingDeflection + headingSpread * NormalDistribution.inverse(Math.random());
		double randomStepLength = stepLength + stepSpread * NormalDistribution.inverse(Math.random());
		
		return new Particle(x, y, randomHeading, randomStepLength, areaIdx, intWeight);
	}



	public double[] getState() {
		return state;
	}


	public int getAreaIndex() {
		return areaIndex;
	}

	public double getDoubleWeight() {
		return doubleWeight;
	}

	public int getIntWeight() {
		return intWeight;
	}

	public void setDoubleWeight(double doubleWeight) {
		this.doubleWeight = doubleWeight;
	}

	public void setIntWeight(int intWeight) {
		this.intWeight = intWeight;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}



	public String toString() {
		return "particle(" + Arrays.toString(state) + ", " + areaIndex + ")";
	}


	public Particle copy(int weight) {
		return new Particle(state[0], state[1], state[2], state[3], areaIndex, weight);
	}
}
