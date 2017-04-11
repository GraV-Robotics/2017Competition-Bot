package fsm;

import java.util.ArrayList;
/**
 * 
 * @author Gregory Tracy
 *
 */
public class SingleStateSelector extends StateSelector {
	
	private State state;
	
	public SingleStateSelector(State state){
		this.state = state;
	}

	@Override
	public ArrayList<State> getStates() {
		ArrayList<State> output = new ArrayList<>();
		output.add(this.state);
		return output;
	}
	
	
}
