package roadSim;

import repast.simphony.space.continuous.ContinuousSpace;

public class Vehicle {
	
	private int maxSpeed, acc;
	private ContinuousSpace<Object> road;
	
	public Vehicle(ContinuousSpace<Object> road, int mS, int a) {
		this.road = road;
		this.maxSpeed = mS;
		this.acc = a;
	}
	
	public void step() {
		// Check for nearby cars
		
		// Drive
	}
	
	public void Drive() {
		// Takes distance to nearest vehicles
		
		// Set acceleration
		
		// Update velocity
		
		// Update postition
	}

}
