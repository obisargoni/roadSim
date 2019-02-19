package roadSim;

import java.awt.Color;

import repast.simphony.visualizationOGL2D.DefaultStyleOGL2D;

public class SignalStyler extends DefaultStyleOGL2D {

	/* (non-Javadoc)
	 * @see repast.simphony.visualizationOGL2D.DefaultStyleOGL2D#getColor(java.lang.Object)
	 */
	@Override
	public Color getColor(Object agent) {
		// Check the state of the agent and colour it accordingly
		if (agent instanceof Signal) {
			boolean sigState = ((Signal)agent).getState();
			
			if (sigState == true) {
				// Colour the agent green
				return Color.GREEN;
			}
			else if (sigState == false) {
				// Colour red
				return Color.RED;
			}
		}
		return super.getColor(agent);
	}

}
