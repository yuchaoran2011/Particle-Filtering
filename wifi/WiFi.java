package pf.wifi;

import pf.distr.NormalDistribution;

public class WiFi {
	final static double SIGMA = 0.5;

	// (x1, y1) is the particle's current coordinates
	// (y1, y2) is the reported estimation from Wi-Fi localization server
	public static double observationProb(double x1, double y1, double x2, double y2) {
		return NormalDistribution.pdf(0.5f, (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
	}
}