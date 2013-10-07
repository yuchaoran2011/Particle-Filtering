package pf;

import pf.floors.*;
import pf.particle.ParticlePosition;
import pf.particle.ParticlePosition.ParticleGenerationMode;

import py4j.GatewayServer;

public class TestDriver {
	public static void main(String[] args) {
		GatewayServer gatewayServer = new GatewayServer(new TestDriver());
        gatewayServer.start();
        System.out.println("Gateway Server Started");
	}
}