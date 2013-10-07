package pf.particle;

import java.util.*;

import pf.distr.NormalDistribution;
import pf.floors.*;
import pf.particle.*;
import pf.utils.*;


/**
 * ParticlePosition class. Represents a model of position computation by means
 * of a particle filter.
 */
public class ParticlePosition implements PositionModel {
	private Set<Particle> particles;
	private Area mArea;
	private double[] mCloudAverageState;

	private static final int DEFAULT_PARTICLE_COUNT = 100;
	private static final double DEFAULT_ALPHA = .99f;
	private static final double HEADING_SIGMA = (double) (Math.PI * 10f / 180f);
	private static final double DEFAULT_STEP_LENGTH = .70f;
	private static final double DEFAULT_STEP_LENGTH_SPREAD = .1f * DEFAULT_STEP_LENGTH;
	private static final double DEFAULT_HEADING_SPREAD = 10f / 180f * (double) Math.PI;
	private static final int DEFAULT_WEIGHT = 1000;
	private static final double NANO = Math.pow(10, 9);

	private double mPositionSigma = 1.0f;
	private double mHeading; // yaw 
	private double[] mCoords;
	private Rectangle mBox;

	private int mNumberOfParticles;
	private double mStepProbability;
	private double mStepLength;
	private double mStepLengthSpread;
	private double mHeadingSpread;
	private boolean mCheckWallsCollisions = true;

	private ArrayList<Line2D> wallCache;

	public enum ParticleGenerationMode { GAUSSIAN, UNIFORM }
	private ParticleGenerationMode mParticleGeneration = ParticleGenerationMode.GAUSSIAN;


	public ParticlePosition() {
		this(-30.0, 25.0, 1.0);
	}


	public ParticlePosition(double posX, double posY) {
		this(posX, posY, 1.0);
	}

	public ParticlePosition(double x, double y, double sigma) {
		int numberOfParticles = DEFAULT_PARTICLE_COUNT;
		particles = new HashSet<Particle>(numberOfParticles);
		while (numberOfParticles > 0) {
			particles.add(Particle.polarNormalDistr(x, y, sigma, 
					mHeadingSpread, mStepLength,
					mStepLengthSpread, DEFAULT_WEIGHT));
			numberOfParticles--;
		}
		mArea = new EmptyArea();
		mCoords = new double[4];
		mCloudAverageState = new double[4];
		mCloudAverageState[0] = x;
		mCloudAverageState[1] = y;
		wallCache = new ArrayList<Line2D>();
		removeInvalidParticles();
	}


	public void setArea(Area area) { mArea = area; }
	public Area getArea() { return mArea; }


	@Override
	public void setPosition(double posX, double posY, int weight) {
		if (mParticleGeneration == ParticleGenerationMode.GAUSSIAN) {
			setPositionNormDistr(posX, posY, mPositionSigma, weight);
		} else {
			setPositionEvenlySpread(posX, posY, 2*mPositionSigma, 2*mPositionSigma, DEFAULT_WEIGHT);
		}
	}

	public void setPositionNormDistr(double posX, double posY, double sigma, int weight) {
		int number = mNumberOfParticles;
		particles = new HashSet<Particle>(mNumberOfParticles);
		while (number > 0) {
			particles.add(Particle.polarNormalDistr(posX, posY, sigma,
					mHeadingSpread, mStepLength,
					mStepLengthSpread, weight));
			number--;
		}
		computeCloudAverageState();
	}


	public void setPositionEvenlySpread(double posX, double posY, double spreadX, double spreadY, int weight) {
		int number = mNumberOfParticles;
		particles = new HashSet<Particle>(mNumberOfParticles);
		while (number > 0) {
			particles.add(Particle.evenSpread(posX, posY, spreadX, spreadY,
					mHeadingSpread, mStepLength,
					mStepLengthSpread, weight));
			number--;
		}
		computeCloudAverageState();
	}



	/**
	 * Box Rectangle of the bounding box of all particles.
	 * 
	 * @return
	 */
	public Rectangle getBox() {
		return mBox;
	}

	public AreaLayerModel getWallsModel() {
		return mArea.getWallsModel();
	}

	private void adjustHeadingDistribution() {
		double stdDeviation = 0.0;
		for (Particle particle : particles) {
			stdDeviation += particle.state[2] - mCoords[2];
		}
		for (Particle particle : particles) {
			particle.state[2] = mCoords[2] + mHeadingSpread
					* NormalDistribution.inverse(Math.random());
		}
	}



	public void onStep(double hdg, double length, double diff) {		
		if (length > 0.0) {
			System.out.println("onStep(hdg: " + hdg + ", length: " + length);

			HashSet<Particle> living = new HashSet<Particle>(particles.size());
			for (Particle particle : particles) {
				Particle newParticle = updateParticle(particle, hdg, length);
				if (newParticle.getWeight() > 0) {
					living.add(newParticle);
				}
			}
			if (living.size() > 0.1 * particles.size()) {
				removeInvalidParticles();
				particles.clear();
				particles.addAll(living);
				System.out.println("No. particles = " + particles.size());
				if (particles.size() < 0.5*DEFAULT_PARTICLE_COUNT) {
					System.out.println("Too few particles! Resampling...");
					resample();
					System.out.println("After resampling: No. particles = " + particles.size());
				}
			}
			else {
				System.out.println("All particles collided with walls and none was left!");
				System.out.println("Generating new particles at most recent valid position!");

				int numberOfParticles = DEFAULT_PARTICLE_COUNT;
				particles = new HashSet<Particle>(numberOfParticles);
				while (numberOfParticles > 0) {
					particles.add(Particle.polarNormalDistr(mCloudAverageState[0], mCloudAverageState[1], 0.5, 
							mHeadingSpread, mStepLength,
							mStepLengthSpread, DEFAULT_WEIGHT));
					numberOfParticles--;	
				}	
				mNumberOfParticles = DEFAULT_PARTICLE_COUNT;

				removeInvalidParticles();
			}	
			computeCloudAverageState();
			// adjustStepLengthDistribution();
			// adjustHeadingDistribution();
		}
	}



	private void removeInvalidParticles() {
		ArrayList<Particle> bad = new ArrayList<Particle>();
		outer:
		for (Particle p : particles) {
			Line2D l1 = new Line2D(mCloudAverageState[0], mCloudAverageState[1], p.getX(), p.getY());
			Collection<Line2D> walls = mArea.getWallsModel().getWalls();
			for (Line2D cachedLines: wallCache) {
				if (cachedLines.intersect(l1)) {
					bad.add(p);
					mNumberOfParticles--;
					continue outer;
				}
			}
			for (Line2D l2 : walls) {
				if (l2.intersect(l1)) {
					bad.add(p);
					mNumberOfParticles--;
					if (wallCache.size() == 10) {
						wallCache.set((new Random()).nextInt(10), l2);
					}
					else {
						wallCache.add(l2);
					}
					break;
				}
			}
		}
		particles.removeAll(bad);
		System.out.println(bad.size() + " new particles were removed because they are in invalid regions!");
		System.out.println(particles.size() + " valid particles remain.");
	}





	public void onStep(double alpha, double hdg, double hdgSpread,
			double length, double lengthSpread) {
		if (length > 0.0) {
			System.out.println("onStep(hdg: " + hdg + ", length: " + length + ", hdgSpread: " + hdgSpread + 
				", lengthSpread: " + lengthSpread);
			HashSet<Particle> living = new HashSet<Particle>();
			for (Particle particle : particles) {
				if (Math.random() > alpha) {
					living.add(particle);
					continue;
				}
				double lengthFactor = 1.0 + lengthSpread * NormalDistribution.inverse(Math.random()) / length;
				Particle newParticle = updateParticle(particle, hdg + hdgSpread
						* NormalDistribution.inverse(Math.random()), lengthFactor);
				if (newParticle.getWeight() > 0) {
					living.add(particle);
				}
			}
			particles.clear();
			particles.addAll(living);

			if (particles.size() < 0.65 * mNumberOfParticles) {
				resample();
			}

			System.out.println("onStep resampling finished: No. particles = " + particles.size());
			computeCloudAverageState();
		}
	}






	private Particle updateParticle(Particle particle, double hdg, double length) {
		Random ran = new Random();
		double[] state = particle.getState();

		// Gaussian noise: http://www.javamex.com/tutorials/random_numbers/gaussian_distribution_2.shtml
		double deltaX = length * Math.sin(hdg); //+ ran.nextGaussian() * 0.4;
		double deltaY = length * Math.cos(hdg); //+ ran.nextGaussian() * 0.4;
		Line2D trajectory = new Line2D(state[0], state[1], state[0] + deltaX, state[1] + deltaY);

		if (mArea != null) {
			// wall collision
			if (mCheckWallsCollisions) {
				for (Line2D wall: wallCache) {
					if (trajectory.intersect(wall)) {
						mNumberOfParticles--;	
						return particle.copy(0);   // Return a dead particle of weight 0
					}
				} 
				Collection<Line2D> walls = mArea.getWallsModel().getWalls();
				for (Line2D wall: walls) {
					if (trajectory.intersect(wall)) {
						mNumberOfParticles--;
						if (wallCache.size() == 10) {
							wallCache.set((new Random()).nextInt(10), wall);
						}
						else {
							wallCache.add(wall);
						}
						return particle.copy(0);   // Return a dead particle of weight 0
					}
				} 
			}
		}
		state[0] += deltaX;
		state[1] += deltaY;
		return new Particle(state[0], state[1], hdg, length, particle.getWeight());
	}



	public void onRssUpdate(double sigma, double x, double y) {
		System.out.println("onRssUpdate()");
		HashSet<Particle> living = new HashSet<Particle>();
		if (particles.isEmpty()) {
			System.out.println("Particles don't exist! Regenerating particles based on WiFi location");

			int numberOfParticles = DEFAULT_PARTICLE_COUNT;
			particles = new HashSet<Particle>(numberOfParticles);
			while (numberOfParticles > 0) {
				particles.add(Particle.polarNormalDistr(x, y, 1, 
							mHeadingSpread, mStepLength,
							mStepLengthSpread, DEFAULT_WEIGHT));
				numberOfParticles--;	
				}	
			mNumberOfParticles = DEFAULT_PARTICLE_COUNT;
		}
		else {
			for (Particle particle : particles) {
				Particle newParticle = particle.copy(particle.getWeight());

				double result = (particle.getX()-x)*(particle.getX()-x)+(particle.getY()-y)*(particle.getY()-y);
				double firstPart = 1.0/(Math.sqrt(2.0*Math.PI) * sigma);
				double secondPart = Math.exp(-result/(2.0 * sigma * sigma));
				double finalResult = firstPart * secondPart;

				newParticle.setWeight((int)(Math.round(particle.getWeight()*finalResult)));

				if (newParticle.getWeight() >= 1) {
					living.add(newParticle);
				}
			}
			if (living.size() > 0.1 * particles.size()) {
				particles.clear();
				particles.addAll(living);
				System.out.println(particles.size());
				System.out.println("WiFi/Image update finished! Resampling...");
				resample();
				System.out.println("After resampling: No. particles = " + particles.size());
			}
			else {
				System.out.println("WiFi observation eliminated too many particles! Regenerating particles at WiFi location!");

				particles.clear();
				int numberOfParticles = DEFAULT_PARTICLE_COUNT;
				particles = new HashSet<Particle>(numberOfParticles);
				while (numberOfParticles > 0) {
					particles.add(Particle.polarNormalDistr(x, y, 1, 
							mHeadingSpread, mStepLength,
							mStepLengthSpread, DEFAULT_WEIGHT));
					numberOfParticles--;	
				}	
				mNumberOfParticles = DEFAULT_PARTICLE_COUNT;
			}
		}	
		computeCloudAverageState();
	}





	private void computeCloudAverageState() {
		mCloudAverageState[0] = mCloudAverageState[1] = mCloudAverageState[2] = mCloudAverageState[3] = 0.0;
		double[] max = new double[] { Double.NEGATIVE_INFINITY,
				Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY,
				Double.NEGATIVE_INFINITY };
		double[] min = new double[] { Double.POSITIVE_INFINITY,
				Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
				Double.POSITIVE_INFINITY };
		for (Particle particle : particles) {
			for (int i = 0; i < particle.getState().length-1; i++) {
				mCloudAverageState[i] += particle.getState()[i];
				min[i] = Math.min(min[i], particle.getState()[i]);
				max[i] = Math.max(max[i], particle.getState()[i]);
			}
		}
		for (int i = 0; i < 4; i++) {
			mCloudAverageState[i] /= particles.size();
		}

		System.out.println("Avg x: " + mCloudAverageState[0] + " Avg y: " + mCloudAverageState[1] + "\n\n");

		mBox = new Rectangle(min[0] - 2 * DEFAULT_STEP_LENGTH, min[1] - 2
				* DEFAULT_STEP_LENGTH, max[0] + 2 * DEFAULT_STEP_LENGTH, max[1]
				+ 2 * DEFAULT_STEP_LENGTH);

		long timestamp = System.nanoTime();
		if (mArea != null) {
			mArea.setBoundingBox(mBox);
		}
	}




	/**
	 * Resample particles. A particle is selected at a frequency proportional to its weight.
	 * Newly generated particles all have DEFAULT_WEIGHT.
	 */
	private void resample() {

		ArrayList<Particle> temp = new ArrayList<Particle>();
		ArrayList<Double> freq = new ArrayList<Double>();
		temp.addAll(particles);
		System.out.println("particles size: "+particles.size());
		particles.clear();
		mNumberOfParticles = 0;

		int sum = 0;
		for (Particle p: temp) {
			sum += p.getWeight();
		}
		double cumulativeFreq = 0;
		for (int i=0; i<temp.size(); i++) {
			cumulativeFreq += temp.get(i).getWeight() / (double)sum;
			freq.add(new Double(cumulativeFreq));
		}

		Random generator = new Random();
		double r = generator.nextDouble();

		for (int i=0; i<DEFAULT_PARTICLE_COUNT; i++) {
			for (int j=0; j<temp.size(); j++) {
				if (r >= freq.get(j)) {
					particles.add(temp.get(j).copy(DEFAULT_WEIGHT));
					mNumberOfParticles++;
					break;
				}
			}
			r = generator.nextDouble();
		}
	}





	public Collection<Particle> getParticles() { return particles; }




	@Override
	public String toString() {
		return "particle(count: " + particles.size() + ", hdg:"
				+ String.format("%.2f", 180 * mHeading / Math.PI) + ", x:"
				+ String.format("%.2f", mCoords[0]) + ", y:"
				+ String.format("%.2f", mCoords[1]) + ", hdg:"
				+ String.format("%.2f", mCoords[2]) + ", step:"
				+ String.format("%.2f", mCoords[3]);
	}




	private double mLastNotifiedHeading;
	private static final double HEADING_DIFF_THRESHOLD = (double) (5f / 180f * Math.PI);




	public double[] getCoords() {
		mCoords[0] = mCloudAverageState[0];
		mCoords[1] = mCloudAverageState[1];
		mCoords[2] = mCloudAverageState[2];
		mCoords[3] = mCloudAverageState[3];
		return mCoords;
	}


	public String getCenter() {
		return mCloudAverageState[0] + " " + mCloudAverageState[1];
	}

	
	public double getPrecision() {
		double sdX = 0.0;
		double sdY = 0.0;
		for (Particle particle: particles) {
			sdX += (particle.state[0] - mCoords[0]) * (particle.state[0] - mCoords[0]);
			sdY += (particle.state[1] - mCoords[1]) * (particle.state[1] - mCoords[1]);
		}
		sdX = Math.sqrt(sdX / particles.size());
		sdY = Math.sqrt(sdY / particles.size());
		return Math.max(sdX, sdY);
	}

	public double getHeading() {
		return mHeading;
	}
}
