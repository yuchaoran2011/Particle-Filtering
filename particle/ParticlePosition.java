package pf.particle;

import java.util.*;

import pf.distr.NormalDistribution;
import pf.floors.*;
import pf.particle.*;
import pf.utils.*;


public class ParticlePosition {
	private Set<Particle> particles;
	private Area mArea;
	private double[] mCloudAverageState;

	private static final int DEFAULT_PARTICLE_COUNT = 100;
	private static final int DEFAULT_WEIGHT = 1000;

	private double mPositionSigma = 1.0f;
	private int mNumberOfParticles;

	private ArrayList<Line2D> wallCache;

	public enum ParticleGenerationMode { GAUSSIAN, UNIFORM }
	private ParticleGenerationMode mParticleGeneration = ParticleGenerationMode.GAUSSIAN;


	public ParticlePosition(double posX, double posY) {
		this(posX, posY, 1.0);
	}

	public ParticlePosition(double x, double y, double sigma) {
		int numberOfParticles = DEFAULT_PARTICLE_COUNT;
		particles = new HashSet<Particle>(numberOfParticles);
		while (numberOfParticles > 0) {
			particles.add(Particle.polarNormalDistr(x, y, sigma, DEFAULT_WEIGHT));
			numberOfParticles--;
		}
		mArea = new EmptyArea();
		mCloudAverageState = new double[2];
		mCloudAverageState[0] = x;
		mCloudAverageState[1] = y;
		wallCache = new ArrayList<Line2D>();
		removeInvalidParticles();
	}


	public void setArea(Area area) { mArea = area; }
	public Area getArea() { return mArea; }


	public void setPositionEvenlySpread(double posX, double posY, double spreadX, double spreadY, int weight) {
		int number = mNumberOfParticles;
		particles = new HashSet<Particle>(mNumberOfParticles);
		while (number > 0) {
			particles.add(Particle.evenSpread(posX, posY, spreadX, spreadY, weight));
			number--;
		}
		computeCloudAverageState();
	}


	public AreaLayerModel getWallsModel() {
		return mArea.getWallsModel();
	}


	public void onStep(double hdg, double length) {		
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
					particles.add(Particle.polarNormalDistr(mCloudAverageState[0], mCloudAverageState[1], 0.5, DEFAULT_WEIGHT));
					numberOfParticles--;	
				}	
				mNumberOfParticles = DEFAULT_PARTICLE_COUNT;

				removeInvalidParticles();
			}	
		}
	}


	private void removeInvalidParticles() {
		ArrayList<Particle> bad = new ArrayList<Particle>();
		outer:
		for (Particle p : particles) {
			Line2D l1 = new Line2D(mCloudAverageState[0], mCloudAverageState[1], p.getX(), p.getY());
			Collection<Line2D> walls = mArea.getWallsModel().getWalls();
			for (Line2D cachedLine: wallCache) {
				if (cachedLine.intersect(l1)) {
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
				}
			}
		}
		particles.removeAll(bad);
		System.out.println(bad.size() + " new particles were removed because they are in invalid regions!");
		System.out.println(particles.size() + " valid particles remain.");
	}



	private Particle updateParticle(Particle particle, double hdg, double length) {
		Random ran = new Random();

		// Gaussian noise: http://www.javamex.com/tutorials/random_numbers/gaussian_distribution_2.shtml
		double deltaX = length * Math.sin(hdg); //+ ran.nextGaussian() * 0.4;
		double deltaY = length * Math.cos(hdg); //+ ran.nextGaussian() * 0.4;
		double oldX=particle.getX(), oldY=particle.getY();
		Line2D trajectory = new Line2D(oldX, oldY, oldX + deltaX, oldY + deltaY);

		if (mArea != null) {
			// wall collision
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
		return new Particle(oldX+deltaX, oldY+deltaY, particle.getWeight());
	}



	public void onRssImageUpdate(double sigma, double x, double y) {
		System.out.println("onRssImageUpdate()");
		HashSet<Particle> living = new HashSet<Particle>();
		if (particles.isEmpty()) {
			System.out.println("Particles don't exist! Regenerating particles based on WiFi location");

			int numberOfParticles = DEFAULT_PARTICLE_COUNT;
			particles = new HashSet<Particle>(numberOfParticles);
			while (numberOfParticles > 0) {
				particles.add(Particle.polarNormalDistr(x, y, 1, DEFAULT_WEIGHT));
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
				System.out.println("Too many particles were eliminated! Regenerating particles at WiFi location!");

				particles.clear();
				int numberOfParticles = DEFAULT_PARTICLE_COUNT;
				particles = new HashSet<Particle>(numberOfParticles);
				while (numberOfParticles > 0) {
					particles.add(Particle.polarNormalDistr(x, y, 1, DEFAULT_WEIGHT));
					numberOfParticles--;	
				}	
				mNumberOfParticles = DEFAULT_PARTICLE_COUNT;
			}
		}	
	}




	private void computeCloudAverageState() {
		mCloudAverageState[0] = mCloudAverageState[1] = 0.0;
		int totalWeight = 0;
		for (Particle particle : particles) {
			mCloudAverageState[0] += particle.getX() * particle.getWeight();
			mCloudAverageState[1] += particle.getY() * particle.getWeight();
			totalWeight += particle.getWeight();
		}
		for (int i = 0; i < 2; ++i) {
			mCloudAverageState[i] /= (double)totalWeight;
		}

		System.out.println("Avg x: " + mCloudAverageState[0] + " Avg y: " + mCloudAverageState[1] + "\n\n");
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
		for (int i=0; i<temp.size(); ++i) {
			cumulativeFreq += temp.get(i).getWeight() / (double)sum;
			freq.add(new Double(cumulativeFreq));
		}

		Random generator = new Random();
		double r = generator.nextDouble();

		for (int i=0; i<DEFAULT_PARTICLE_COUNT; ++i) {
			for (int j=0; j<temp.size(); ++j) {
				if (r >= freq.get(j)) {
					particles.add(temp.get(j).copy(DEFAULT_WEIGHT));
					mNumberOfParticles++;
					break;
				}
			}
			r = generator.nextDouble();
		}
	}


	public Collection<Particle> getParticles() { 
		return particles; 
	}


	public String getCenter() {
		computeCloudAverageState();
		return mCloudAverageState[0] + " " + mCloudAverageState[1];
	}

	
	public double getPrecision() {
		double sdX = 0.0;
		double sdY = 0.0;
		for (Particle particle: particles) {
			sdX += (particle.getX() - mCloudAverageState[0]) * (particle.getX() - mCloudAverageState[0]);
			sdY += (particle.getY() - mCloudAverageState[1]) * (particle.getY() - mCloudAverageState[1]);
		}
		sdX = Math.sqrt(sdX / particles.size());
		sdY = Math.sqrt(sdY / particles.size());
		return Math.max(sdX, sdY);
	}
}
