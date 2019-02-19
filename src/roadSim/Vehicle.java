package roadSim;

import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;

public class Vehicle {
	
	private int maxSpeed, followDist; // The distance from a vehicle ahead at which the agent adjusts speed to follow
	private double speed, acc;
	private double stepToTimeRatio = 1; // This will need to be set at a high level to control the graularity of time
	private ContinuousSpace<Object> road;
	
	public Vehicle(ContinuousSpace<Object> road, int mS, int flD, double a, double s) {
		this.road = road;
		this.maxSpeed = mS;
		this.acc = a;
		this.speed = s;
		this.followDist = flD; 
	}
	
	public double getSpeed() {
		return this.speed;
	}
	
	/* Default behaviour, shuffle = true, randomised the scheduling
	 * of collections of agents. In the case of a road, the process may not be random
	 * since the vehicle in the front might have priority.
	 */
	@ScheduledMethod(start = 1, interval = 1, shuffle = true)
	public void step() {
		// Check for nearby cars
		Vehicle nearestVehicle = getNearestVehicle();
		
		// Drive
		drive(nearestVehicle);
		//moveForward();
	}
	
	/* Get the vehicle agent that is closest in front of this vehicle agent.
	 * 
	 * How to handle wrap around?
	 * Is a minimum distance necessary?
	 */
	public Vehicle getNearestVehicle() {
		// Initialise variables
		Vehicle nearestVehicle = null;
		double minSep = this.road.getDimensions().getWidth(); // Initialise the minimum separation as whole road width
		
		// Get the vehicles that is closest in front to this agent
		// There is probably a more direct way of iterating over the Vehicle agents
		for (Object o: this.road.getObjects()) {
			if (o instanceof Vehicle) {
				double sep = road.getLocation(o).getX() - road.getLocation(this).getX();
				// Don't consider vehicles that are behind the target vehicle
				if (sep < minSep & Math.signum(sep) == 1.0) {
					nearestVehicle = (Vehicle) o;
					minSep = sep;
				}
			}

		}
		
		return nearestVehicle;
	}
	
	/* 
	 * Updates the vehicle's speed using 
	 * the General Motors car following model described here:
	 * {@link https://nptel.ac.in/courses/105101008/downloads/cete_14.pdf}
	 * In future this will be revised to ensure a good academic car following
	 * model is used
	 * 
	 * @param vehcileInFront The vehicle immediately in front of the ego vehicle
	 * @return The new speed
	 */
	public double setSpeed(Vehicle vehicleInFront) {
		// Update velocity
		this.speed = this.speed + this.acc * stepToTimeRatio;
		
		return this.speed;
		
	}
	
	/* Simply sets speed to be the speed of the vehicle in front
	 * accounting for the acceleration or deceleration required to get 
	 * to that speed in the next timestep. This assumes a fixed acceleration.
	 * 
	 * @param vehicleInFront Vehicle. The vehicle agent in front of this vehicle agent
	 * @return Double. The speed set for this vehicle
	 */
	public double setSpeedSimple(Vehicle vehicleInFront) {
		
		// Set speed so that after one time step it will be the same as the car in front
		if (vehicleInFront != null) {
			// Get location of vehicle in front
			double vifXCoord = vehicleInFront.road.getLocation(vehicleInFront).getX();
			
			// If the vehicle in front is sufficiently cloes, adjust speed
			if ((vifXCoord - this.road.getLocation(this).getX()) < this.followDist) {
				// Get speed of vehicle in front
				double infSpeed = vehicleInFront.getSpeed();
				this.speed = infSpeed - (this.acc * stepToTimeRatio);
			}
			else {
				// If the vehicle in front is not close enough speed up
				this.speed = this.speed + (this.acc * stepToTimeRatio);
			}
		} 
		// If there is no vehicle in front just speed up
		else {
			this.speed = this.speed + (this.acc * stepToTimeRatio);
		}
		
		// Enforce speed limits
		if (this.speed > this.maxSpeed) {
			this.speed = this.maxSpeed;
		}
		// Min speed is zero (no-reversing)
		if (this.speed < 0) {
			this.speed = 0;
		}
		return this.speed;
		
	}
	
	/* Updates the vehicle's acceleration using
	 * the General Motors car following model described here:
	 * {@link https://nptel.ac.in/courses/105101008/downloads/cete_14.pdf}.
	 * This model might not be suitable for this simple exercise.
	 * 
	 * @param vehicleInFront The vehicle immediately in front of the ego vehicle
	 * @return The updated acceleration
	 */
	public double setAcc(Vehicle vehicleInFront) {
		// Update acceleration based on the position and velocity of the vehicle in front.
		// Only do this if there is a vehicle in front to follow
		if (vehicleInFront != null) {
			int alpha, m, l;
			alpha = m = l = 1; // Parameters for the car following model. Needs refactor.
			this.acc = (((alpha * Math.pow(this.speed,m)) / Math.pow(this.road.getLocation(vehicleInFront).getX() - this.road.getLocation(this).getX(),l)) * (vehicleInFront.getSpeed() - this.speed));
		} else {
			// Set acceleration to zero. This is not realistic, needs refining
			this.acc = 0;
		}
		
		return this.acc;
	}
	
	/* 
	 * Drive the vehicle agent. Set the vehicle agent's speed and update the 
	 * x-coordinate of the vehicle using its current speed and acceleration and
	 * preventing overtaking. Move the vehicle agent to its new location.
	 * 
	 * Prevention of overtaking is not currently working.
	 * 
	 * @param vehicleInFront Vehicle. The vehicle in front of the agent vehicle  
	 * 
	 */
	public void drive(Vehicle vehicleInFront) {
		// Update speed and acceleration
		setSpeedSimple(vehicleInFront);
		//setAcc(vehicleInFront);
		
		// Update position
		NdPoint currPos = this.road.getLocation(this);
		double newXCoord = currPos.getX() + this.speed * stepToTimeRatio + 0.5 * this.acc * Math.pow(stepToTimeRatio, 2);
		
		// Prevent overtaking (currently producing unexpected behaviour)
		//newXCoord = preventOvertake(newXCoord, vehicleInFront);
		
		this.road.moveTo(this, newXCoord, currPos.getY());
		
	}
	
	/*
	 * If the vehicle in front is not null, check if the new x-coordinate
	 * for this agent does not exceed the x-coordinate of the vehicle in front.
	 * If it does, set the new x-coordinate to be slightly behind the vehicle in front.
	 * 
	 *  @param nXC Double. The new x-coordinate of this agent
	 *  @param vIF Vehicle. The vehicle agent in front of this agent.
	 *  @return Double. The x-coordinate to move this agent to that avoids overtaking.
	 */
	public double preventOvertake(double nXC, Vehicle vIF) {
		// Prevent vehicle from moving past the vehicle in front by checking that the
		if (vIF == null) {
			// There is not vehicle in front so don't need to worry about overtaking
			// What about looping round and overtaking an agent though?
			return nXC;	
		}
		else {
			double vifXC = vIF.road.getLocation(vIF).getX();
			if (nXC > vifXC) {
				// Assumes cars need to be a distance of 1 apart
				nXC = vIF.road.getLocation(vIF).getX() - 1;				
			}
			
			return nXC;
		}	
	}
	
	/*
	* Move the agent in the positive x direction 10 units.
	*/
	public void moveForward() {
		// Get current position
		NdPoint currPos = this.road.getLocation(this);
		this.road.moveTo(this, currPos.getX() + 10, currPos.getY());
	}

}
