package pf;

import pf.floors.AreaBuilder;
import pf.floors.Area;
import pf.particle.ParticlePosition;
import pf.particle.ParticlePosition.ParticleGenerationMode;

import py4j.GatewayServer;


public class TestDriver {


	public static void main(String[] args) {
		
		GatewayServer gatewayServer = new GatewayServer(new TestDriver());
        gatewayServer.start();
        System.out.println("Gateway Server Started");

        
		// Get initial position and feed it into ParticlePosition constructor

        /*
		// Get step length and heading from phone sensors
		double heading = 0.0, stepLength = 0.8; 

		// Update particle positions
		particleCloud.onStep(heading, stepLength);  // Wall collision checking included

		// Get Wi-Fi and image localization results and get probability based on observation model
		float WiFix=0, WiFiy=0, imageX, imageY;
		particleCloud.onRssMeasurementUpdate(WiFix, WiFiy);

		// Reweight particles

		// Resample particles and report location
		//particleCloud.computeCloudAverageState();*/
	}
}