package roadSim;

import repast.simphony.context.Context;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;
import repast.simphony.util.ContextUtils;
import repast.simphony.util.collections.IndexedIterable;

public class Vehicle {
	
	private int maxSpeed, followDist; // The distance from a vehicle ahead at which the agent adjusts speed to follow
	private double speed, acc, dacc, buffer;
	private double stepToTimeRatio = 1; // This will need to be set at a high level to control the graularity of time
	private ContinuousSpace<Object> road;
	
	public Vehicle(ContinuousSpace<Object> road, int mS, int flD, double a, double s) {
		this.road = road;
		this.maxSpeed = mS;
		this.acc = this.dacc = a;
		this.speed = s;
		this.followDist = flD; 
		this.buffer = 2; // Min distance to keep between vehicles or between vehicle and a signal
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
	public double setSpeedFollowing(Vehicle vehicleInFront) {
		
		// Set speed so that after one time step it will be the same as the car in front
		if (vehicleInFront != null) {
			// Get location of vehicle in front
			double vifXCoord = vehicleInFront.road.getLocation(vehicleInFront).getX();
			
			// If the vehicle in front is sufficiently cloes, adjust speed
			if ((vifXCoord - this.road.getLocation(this).getX()) < this.followDist) {
				// Get speed of vehicle in front
				double vifSpeed = vehicleInFront.getSpeed();
				this.speed = vifSpeed - (this.acc * stepToTimeRatio);
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
	
	/*
	 * Set the speed and acceleration of the vehicle agent such that it will
	 * come to a complete stop at the signal.
	 * 
	 *  Doesn't account for leaving space for other cars.
	 */
	public double setSpeedSignal(Signal s, Vehicle vehicleInFront) {
		double d; // initialise the distance the vehicle must stop in
		double sigX = this.road.getLocation(s).getX();
		double vX = this.road.getLocation(this).getX();
		
		if (vehicleInFront == null) {
			d = sigX - vX - this.buffer;
		}
		else {
			double vifX = this.road.getLocation(vehicleInFront).getX();
			
			// Depending on whether the vehicle in front or the signal is closer, set the stopping distance
			if (vifX < sigX) {
				d = vifX - vX - this.buffer;
			}
			else {
				d = sigX - vX - this.buffer;
			}
		}
		

		this.dacc = Math.pow(this.speed, 2) / (2 * d); // Get required deceleration using eqns of constant a
		this.speed = this.speed - (this.dacc * stepToTimeRatio); // Might still need to take account of following the car ahead?
		
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
		
		// Check for a traffic signal
		boolean sigState = checkSignal();
		NdPoint currPos = this.road.getLocation(this);
		double newXCoord = currPos.getX(); // Initialise new x coordinate as the current position
		
		if (sigState == true) {
			// Continue driving by following car ahead
			// Update speed and acceleration
			setSpeedFollowing(vehicleInFront);
			newXCoord = currPos.getX() + this.speed * stepToTimeRatio + 0.5 * this.acc * Math.pow(stepToTimeRatio, 2);

			//setAcc(vehicleInFront);
		} else if (sigState == false) {
			// Set speed based on distance from signal
			Signal sig = getSignal();
			setSpeedSignal(sig, vehicleInFront);
			newXCoord = currPos.getX() + this.speed * stepToTimeRatio - 0.5 * this.dacc * Math.pow(stepToTimeRatio, 2);
		}

		
		// Update position
		
		
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
	
	/*
	 * Get the signal agent in the road continuous space
	 * 
	 * @return Signal. The signal agent
	 */
	public Signal getSignal() {
		Signal sig = null;
		for (Object o: this.road.getObjects()) {
			if (o instanceof Signal) {
				sig = (Signal) o;
			}		
		}
		
		return sig;
	}
	
	/*
	 * Identify the signal agent in the space and how far away it is.
	 * If signal agent is within a threshold distance get the state of the signal
	 * 
	 * @return Boolean. True if there is no need to adjust driving. False if signal means stop.
	 */
	public boolean checkSignal() {
		double threshDist = 5; // Distance at which drivers will alter behaviour depending on signal
		Signal sig = getSignal();

		// First check if vehicle has past the signal, in which case there is no signal to check
		double sigX =  this.road.getLocation(sig).getX();
		double vX = this.road.getLocation(this).getX();
		if (vX > sigX) {
			return true;
		}
		else if ((sigX - vX) < threshDist) {
			return sig.getState();
		}
		else {
			return true;
		}
			
	}
}
