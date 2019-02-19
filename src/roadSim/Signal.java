package roadSim;

import repast.simphony.engine.schedule.ScheduledMethod;

public class Signal {
	
	// The state of the signal will indicate whether cars can pass or not
	private boolean state;
	
	/*
	 * Instance method. Must be passed the state of the signal.
	 * 
	 * @param Boolean. The state of the signal. True means cars can pass. False means cars must come to a stop
	 */
	public Signal(boolean state) {
		this.state = state;
	}
	
	/*
	 * Get the state of the signal.
	 * 
	 * @return Boolean. The state of the signal
	 */
	public boolean getState() {
		return this.state;
	}
	
	/*
	 * Set the state of the Signal.
	 * 
	 * @param Boolean. The state to set the signal to.
	 */
	public void setState(boolean nState) {
		this.state = nState;
	}
	
	/*
	 * Reverse the state of the signal.
	 */
	@ScheduledMethod(start = 100, interval = 100)
	public void switchState() {
		this.setState(!this.state);
	}
}
