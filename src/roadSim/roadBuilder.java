/**
 * 
 */
package roadSim;

import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import repast.simphony.context.Context;
import repast.simphony.context.space.continuous.ContinuousSpaceFactory;
import repast.simphony.context.space.continuous.ContinuousSpaceFactoryFinder;
import repast.simphony.dataLoader.ContextBuilder;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.SimpleCartesianAdder;

/**
 * @author Obi
 * Is this the github one?
 */
public class roadBuilder implements ContextBuilder<Object> {

	/* (non-Javadoc)
	 * @see repast.simphony.dataLoader.ContextBuilder#build(repast.simphony.context.Context)
	 */
	@Override
	public Context build(Context<Object> context) {
		// TODO Auto-generated method stub
		
		context.setId("roadSim");
		
		int spaceWidth = 50;
		int spaceHeight = 50;
		int numVehicles = 10; // Number of vehicles to include in the simulation
		
		// Create continuous space
		ContinuousSpaceFactory spaceFact = ContinuousSpaceFactoryFinder.createContinuousSpaceFactory(null);
		ContinuousSpace <Object > space =spaceFact.createContinuousSpace("space", context ,new  SimpleCartesianAdder <Object>(),
				new  repast.simphony.space.continuous.WrapAroundBorders (),spaceWidth, spaceHeight);
		
		
		// Add the vehicles to the context
		for (int i = 0; i<numVehicles;i++) {
			// Set max speed and acceleration parameters for vehicle
			int mS = 3;
			int flD = 5; // This distance needs to be greater than the maximum distance that can be traveled in a single timestep
			double a = 0.1;
			double s = 0.5; // Initial speed of all vehicles is the same
			double brng = 0; // Sets the direction of travel of the agent vehicle
			context.add(new Vehicle(space, mS, flD, a, s, brng));
		}
		
		// Use list of ints from 0 to spaceWidth as possible positions for the vehicles
		List<Integer> xCoords = IntStream.range(0,spaceWidth).boxed().collect(Collectors.toList());
		Random randx = new Random();
		// Set the position of each vehicle
		for (Object obj:context) {
			int xCoordIndex = randx.nextInt(xCoords.size());
			int xCoord = xCoords.get(xCoordIndex);
			xCoords.remove(xCoordIndex); // To ensure that no two cars are initiated at the same spot
			int yCoord = spaceHeight / 2; // Places vehicles half way up the sapce
			space.moveTo(obj, xCoord,yCoord);
		}
		
		
		// Add a signal to the road - add 75% along road way
		int sigXCoord = (spaceWidth / 4) * 3;
		int sigYCoord = (spaceHeight / 4) * 3;
		Signal sig = new Signal(true);
		context.add(sig);
		space.moveTo(sig, sigXCoord, sigYCoord);
		 
		
		return context;
	}
	
	/*
	 * I don't think creating separate continuous spaces for different roads is the right way to go.
	 * 
	 */
	public ContinuousSpace<Object> createRoad(Context<Object> context, double bearing, int length, int width, int origin) {

		// Create road space
		ContinuousSpaceFactory spaceFact = ContinuousSpaceFactoryFinder.createContinuousSpaceFactory(null);
		ContinuousSpace <Object > road =spaceFact.createContinuousSpace("road", context ,new  SimpleCartesianAdder <Object>(),
				new  repast.simphony.space.continuous.WrapAroundBorders (),length, width);
				
		return road;
		
	}

}
