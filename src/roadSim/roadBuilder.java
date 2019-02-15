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
		
		int roadWidth = 10;
		int roadLength = 50;
		
		// Create road space
		ContinuousSpaceFactory spaceFact = ContinuousSpaceFactoryFinder.createContinuousSpaceFactory(null);
		ContinuousSpace <Object > road =spaceFact.createContinuousSpace("road", context ,new  SimpleCartesianAdder <Object>(),
				new  repast.simphony.space.continuous.WrapAroundBorders (),50, 50);
		
		// Create vehicle agents
		int numVehicles = 10;
		
		// Add them to the context
		for (int i = 0; i<numVehicles;i++) {
			// Set max speed and acceleration parameters for vehicle
			int mS = 30;
			int a = 3;
			context.add(new Vehicle(road, mS,a));
		}
		
		// Use list of ints from 0 to 50 as possible positions for the vehicles
		List<Integer> xCoords = IntStream.range(0,roadLength).boxed().collect(Collectors.toList());
		Random randx = new Random();
		// Set the position of each vehicle
		for (Object obj:context) {
			int xCoordIndex = randx.nextInt(xCoords.size());
			int xCoord = xCoords.get(xCoordIndex);
			xCoords.remove(xCoordIndex); // To ensre that no two cars are initiated at the same spot
			int yCoord = roadWidth / 2;
			road.moveTo(obj, xCoord,yCoord);
		}
		
		
		return context;
	}

}