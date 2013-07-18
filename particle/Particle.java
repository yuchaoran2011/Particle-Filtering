package pf.particle;

import java.util.Arrays;

import pf.distr.NormalDistribution;


public class Particle implements Cloneable {

	double[] state; // x, y, heading, stride length, weight
	double x, y;
	double hdg, stepLength;
	int weight; 

	
	public Particle(double posX, double posY, double heading, double stepLength, int weight) {
		state = new double[] { posX, posY, heading, stepLength, weight };
		x = posX;
		y = posY;
		hdg = heading;
		this.stepLength = stepLength; 
		this.weight = weight;
	}



	// polar version
	public static Particle polarNormalDistr(double meanX, double meanY,
			double sigma, double headingDeflection, double headingSpread,
			double stepLength, double stepSpread, int weight) {

		double angle = 2 * Math.PI * Math.random();
		double distance = sigma * NormalDistribution.inverse(Math.random());
		
		double x = meanX + (distance * Math.cos(angle));
		double y = meanY + (distance * Math.sin(angle));
		
		double randomHeading = headingDeflection + headingSpread * NormalDistribution.inverse(Math.random());
		double randomStepLength = stepLength + stepSpread * NormalDistribution.inverse(Math.random());
		return new Particle(x, y, randomHeading, randomStepLength, weight);
	}



	public static Particle evenSpread(double meanX, double meanY, double sizeX, double sizeY,
			double headingDeflection, double headingSpread,
			double stepLength, double stepSpread, int weight) {
		
		
		double x = meanX + ((Math.random() - 0.5) *  sizeX);
		double y = meanY + ((Math.random() - 0.5) * sizeY);
		
		double randomHeading = headingDeflection + headingSpread * NormalDistribution.inverse(Math.random());
		double randomStepLength = stepLength + stepSpread * NormalDistribution.inverse(Math.random());
		
		return new Particle(x, y, randomHeading, randomStepLength, weight);
	}



	public double[] getState() {
		return state;
	}


	public int getWeight() {
		return weight;
	}

	public void setWeight(int weight) {
		this.weight = weight;
	}

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}



	public String toString() {
		return "particle(" + Arrays.toString(state) + ")";
	}



	// Can be used to construct a nearly dead particle (a particle that just crossed a wall).
	public Particle copy(int weight) {
		return new Particle(state[0], state[1], state[2], state[3], weight);
	}
}
