package roadSim;

import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;
import repast.simphony.space.grid.Grid;
import repast.simphony.space.grid.GridPoint;

public class Vehicle {

	private int maxSpeed, followDist, sigIdDist; // The distance from a vehicle ahead at which the agent adjusts speed
													// to follow
	private double speed, acc, dacc, bearing, buffer;
	private double stepToTimeRatio = 1; // This will need to be set at a high level to control the graularity of time
	private ContinuousSpace<Object> space;
	private Grid<Object> grid;

	public Vehicle(ContinuousSpace<Object> space, Grid<Object> grid, int mS, int flD, int sIdD, double a, double s,
			double brng) {
		this.space = space;
		this.grid = grid;
		this.maxSpeed = mS;
		this.acc = this.dacc = a;
		this.speed = s;
		this.followDist = flD;
		this.sigIdDist = sIdD;
		this.bearing = brng;
		this.buffer = 2; // Min distance to keep between vehicles or between vehicle and a signal
	}

	public double getSpeed() {
		return this.speed;
	}

	/*
	 * Default behaviour, shuffle = true, randomised the scheduling of collections
	 * of agents. In the case of a space, the process may not be random since the
	 * vehicle in the front might have priority.
	 */
	@ScheduledMethod(start = 1, interval = 1, shuffle = false)
	public void step() {
		// Check for nearby cars
		Vehicle vehicleInFront = getVehicleInFront();

		// Drive
		drive(vehicleInFront);
		// moveForward();
	}

	/*
	 * Get the vehicle agent that is closest in front of this vehicle agent.
	 * 
	 * How to handle wrap around? Is a minimum distance necessary?
	 */
	public Vehicle getVehicleInFront() {

		// Initialise variables
		Vehicle vehicleInFront = null;
		double minSep = this.space.getDimensions().getWidth(); // Initialise the minimum separation as whole space width

		// Iterate over the coordinates of grid cells ahead of this agent vehicle
		GridPoint thisPt = grid.getLocation(this);
		for (int i = thisPt.getX(); i <= Math.min(this.space.getDimensions().getWidth(),
				thisPt.getX() + this.followDist); i++) {
			Iterable<Object> vIt = grid.getObjectsAt(i, thisPt.getY());

			// Iterate over objects at grid location (iterable could be empty)
			for (Object o : vIt) {
				/*
				 * Get the position of the vehicle and calculate the separation. This distance
				 * is always positive, but this shouldn't matter since the original search only
				 * considered vehicles in the positive x direction.
				 */
				double sep = space.getDistance(space.getLocation(this), space.getLocation(o));
				// Don't consider vehicles that are behind the target vehicle
				if (sep < minSep & Math.signum(sep) == 1.0) {
					vehicleInFront = (Vehicle) o;
					minSep = sep;
				}
			}
		}

		return vehicleInFront;
	}

	/*
	 * Updates the vehicle's speed using the General Motors car following model
	 * described here: {@link
	 * https://nptel.ac.in/courses/105101008/downloads/cete_14.pdf} In future this
	 * will be revised to ensure a good academic car following model is used
	 * 
	 * @param vehcileInFront The vehicle immediately in front of the ego vehicle
	 * 
	 * @return The new speed
	 */
	public double setSpeed() {
		// Update velocity
		this.speed = this.speed + this.acc * stepToTimeRatio;
		
		enforceSpeedLimit();

		return this.speed;

	}

	/*
	 * Simply sets speed to be the speed of the vehicle in front accounting for the
	 * acceleration or deceleration required to get to that speed in the next
	 * timestep. This assumes a fixed acceleration.
	 * 
	 * @param vehicleInFront Vehicle. The vehicle agent in front of this vehicle
	 * agent
	 * 
	 * @return Double. The speed set for this vehicle
	 */
	public double setSpeedFollowing(Vehicle vehicleInFront) {

		/*
		 * Set speed so that after one time step it will be the same as the car in front
		 * The vehicle in front is not null only if it is within the following distance
		 * of this agent vehicle.
		 */
		if (vehicleInFront != null) {
			// Get speed of vehicle in front
			double vifSpeed = vehicleInFront.getSpeed();
			this.speed = vifSpeed - (this.acc * stepToTimeRatio);

		}
		// If there is no vehicle in front just speed up
		else {
			this.speed = this.speed + (this.acc * stepToTimeRatio);
		}

		enforceSpeedLimit();
		return this.speed;
	}

	public double enforceSpeedLimit() {
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
	 * Set the speed and acceleration of the vehicle agent such that it will come to
	 * a complete stop at the signal.
	 * 
	 * Doesn't account for leaving space for other cars.
	 */
	public double setAccSignal(Signal s, Vehicle vehicleInFront) {
		double d; // initialise the distance the vehicle must stop in
		double sigX = this.space.getLocation(s).getX();
		double vX = this.space.getLocation(this).getX();
		
		// How to ensure there is some sort of buffer
		if (vehicleInFront == null) {
			// Follow the signal
			setAccFollowing(s);
		} else {
			double vifX = this.space.getLocation(vehicleInFront).getX();

			// Depending on whether the vehicle in front or the signal is closer, set the
			// stopping distance
			if (vifX < sigX) {
				//d = vifX - vX - this.buffer;
				// Follow the vehicle in front
				setAccFollowing(vehicleInFront);
			} else {
				//d = sigX - vX - this.buffer;
				setAccFollowing(s);
			}
		}
		
		// Assumes vehicles come to complete stop in a single time step - might be cause of bunching
		//this.acc = - Math.pow(this.speed, 2) / (2 * d); // Get required deceleration using eqns of constant a

		return this.acc;
	}

	/*
	 * Updates the vehicle's acceleration using the General Motors car following
	 * model described here: {@link
	 * https://nptel.ac.in/courses/105101008/downloads/cete_14.pdf}. This model
	 * might not be suitable for this simple exercise.
	 * 
	 * @param vehicleInFront The vehicle immediately in front of the ego vehicle
	 * 
	 * @return The updated acceleration
	 */
	public double setAccFollowing(Object objectInFront) {
		// Update acceleration based on the position and velocity of the vehicle in
		// front.
		
		double objV = 0;
		if (objectInFront instanceof Vehicle) {
			objV = ((Vehicle) objectInFront).getSpeed();
		}
		else if (objectInFront instanceof Signal) {
			objV = 0;
		}
	
		// Only do this if there is a vehicle in front to follow
		if (objectInFront != null) {
			int alpha, m, l;
			alpha = 1;
			m = 0;
			l = 0; // Parameters for the car following model. Needs refactor.
			NdPoint thisPt = this.space.getLocation(this);
			NdPoint vifPt = this.space.getLocation(objectInFront);

			// Acceleration is negative since in order to have caught up to car in front
			// will have been traveling faster
			this.acc = (((alpha * Math.pow(this.speed,m)) / Math.pow(this.space.getDistance(thisPt, vifPt),l)) * (objV - this.speed));
			//this.acc = vehicleInFront.getSpeed() - this.getSpeed();
		} else {
			// Set acceleration to zero. This is not realistic, needs refining
			this.acc = 0.1; // Default acceleration
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
		double disp = 0; // Initialise the amount to move the vehicle by

		if (sigState == true) {
			// Continue driving by following car ahead
			// Update acceleration. travel for time step at this acceleration, leading to an updated speed
			setAccFollowing(vehicleInFront);
			disp = this.speed * stepToTimeRatio + 0.5 * this.acc * Math.pow(stepToTimeRatio, 2);
			setSpeed();

			// setAcc(vehicleInFront);
		} else if (sigState == false) {
			// Set speed based on distance from signal
			// In this case signal will be within a certain distance of the vehicle
			Signal sig = getSignal();
			setAccSignal(sig, vehicleInFront);
			disp = this.speed * stepToTimeRatio + 0.5 * this.acc * Math.pow(stepToTimeRatio, 2);
			setSpeed();
		}

		// Move the agent in the space and the grid
		space.moveByVector(this, disp, this.bearing, 0);
		NdPoint thisPt = space.getLocation(this);
		grid.moveTo(this, (int) thisPt.getX(), (int) thisPt.getY());

	}

	/*
	 * If the vehicle in front is not null, check if the new x-coordinate for this
	 * agent does not exceed the x-coordinate of the vehicle in front. If it does,
	 * set the new x-coordinate to be slightly behind the vehicle in front.
	 * 
	 * @param nXC Double. The new x-coordinate of this agent
	 * 
	 * @param vIF Vehicle. The vehicle agent in front of this agent.
	 * 
	 * @return Double. The x-coordinate to move this agent to that avoids
	 * overtaking.
	 */
	public double preventOvertake(double nXC, Vehicle vIF) {
		// Prevent vehicle from moving past the vehicle in front by checking that the
		if (vIF == null) {
			// There is not vehicle in front so don't need to worry about overtaking
			// What about looping round and overtaking an agent though?
			return nXC;
		} else {
			double vifXC = vIF.space.getLocation(vIF).getX();
			if (nXC > vifXC) {
				// Assumes cars need to be a distance of 1 apart
				nXC = vIF.space.getLocation(vIF).getX() - 1;
			}

			return nXC;
		}
	}

	/*
	 * Move the agent in the positive x direction 10 units.
	 */
	public void moveForward() {
		// Get current position
		NdPoint currPos = this.space.getLocation(this);
		this.space.moveTo(this, currPos.getX() + 10, currPos.getY());
	}

	/*
	 * Get the signal agent in the space continuous space
	 * 
	 * @return Signal. The signal agent
	 */
	public Signal getSignal() {
		Signal sig = null;
		for (Object o : this.space.getObjects()) {
			if (o instanceof Signal) {
				sig = (Signal) o;
			}
		}

		return sig;
	}

	/*
	 * Identify the signal agent in the space and how far away it is. If signal
	 * agent is within a threshold distance get the state of the signal
	 * 
	 * @return Boolean. True if there is no need to adjust driving. False if signal
	 * means stop.
	 */
	public boolean checkSignal() {
		double threshDist = 5; // Distance at which drivers will alter behaviour depending on signal
		Signal sig = getSignal();

		// First check if vehicle has past the signal, in which case there is no signal
		// to check
		double sigX = this.space.getLocation(sig).getX();
		double vX = this.space.getLocation(this).getX();
		if (vX > sigX) {
			return true;
		} else if ((sigX - vX) < threshDist) {
			return sig.getState();
		} else {
			return true;
		}

	}
}
