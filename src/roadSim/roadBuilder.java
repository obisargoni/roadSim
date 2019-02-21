/**
 * 
 */
package roadSim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import repast.simphony.context.Context;
import repast.simphony.context.space.continuous.ContinuousSpaceFactory;
import repast.simphony.context.space.continuous.ContinuousSpaceFactoryFinder;
import repast.simphony.context.space.grid.GridFactory;
import repast.simphony.context.space.grid.GridFactoryFinder;
import repast.simphony.dataLoader.ContextBuilder;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.parameter.Parameters;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.SimpleCartesianAdder;
import repast.simphony.space.grid.Grid;
import repast.simphony.space.grid.GridBuilderParameters;
import repast.simphony.space.grid.SimpleGridAdder;
import repast.simphony.space.grid.WrapAroundBorders;

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
		Parameters params = RunEnvironment.getInstance().getParameters();
		
		int numVehicles = params.getInteger("numVehicles"); // Number of vehicles to include in the simulation
		int flD = params.getInteger("followDist");
		int sIdD = params.getInteger("signalDist");
		
		// Create continuous space
		ContinuousSpaceFactory spaceFact = ContinuousSpaceFactoryFinder.createContinuousSpaceFactory(null);
		ContinuousSpace <Object > space =spaceFact.createContinuousSpace("space", context ,new  SimpleCartesianAdder <Object>(),
				new  repast.simphony.space.continuous.WrapAroundBorders (),spaceWidth, spaceHeight);
		
		GridFactory gridFact = GridFactoryFinder.createGridFactory(null);
		Grid<Object> grid = gridFact.createGrid("grid", context, new GridBuilderParameters( new WrapAroundBorders(), 
				new SimpleGridAdder<Object>(), true, spaceWidth,spaceHeight));
		
		// Add the vehicles to the context
		for (int i = 0; i<numVehicles;i++) {
			// Set max speed and acceleration parameters for vehicle
			int mS = 3;
			double a = 0.1;
			double s = 0.5; // Initial speed of all vehicles is the same
			double brng = 0; // Sets the direction of travel of the agent vehicle
			context.add(new Vehicle(space, grid, mS, flD, sIdD, a, s, brng));
		}
		
		// Use list of ints from 0 to spaceWidth as possible positions for the vehicles
		List<Integer> spaceXCoords = IntStream.range(0,spaceWidth).boxed().collect(Collectors.toList());
		Random randx = new Random();
		ArrayList<Integer> agentXCoords = new ArrayList<Integer>();
		for (int i=0; i<numVehicles; i++) {
			// Get a random int within range of space and use this to sample
			int randIndex = randx.nextInt(spaceXCoords.size());
			
			// Add the samples value to the list of xCoords and remove it from the possible set
			agentXCoords.add(spaceXCoords.get(randIndex));
			spaceXCoords.remove(randIndex);
		}
		
		// Sort the random sample of x coords in descending order so that agents are added in order
		Collections.sort(agentXCoords);
		Collections.reverse(agentXCoords);
		
		// Set the position of each vehicle
		int i = 0;
		for (Object obj:context) {
			int xCoord = agentXCoords.get(i);
			int yCoord = spaceHeight / 2; // Places vehicles half way up the space
			space.moveTo(obj, xCoord,yCoord);
			grid.moveTo(obj, xCoord, yCoord);
			i++;
		}
		
		
		// Add a signal to the road - add 75% along road way
		int sigXCoord = (spaceWidth / 4) * 3;
		int sigYCoord = (spaceHeight / 4) * 3;
		Signal sig = new Signal(true);
		context.add(sig);
		space.moveTo(sig, sigXCoord, sigYCoord);
		grid.moveTo(sig, sigXCoord, sigYCoord);
		 
		
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
