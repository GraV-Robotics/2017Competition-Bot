package fsm;

import java.util.ArrayList;
/**
 * 
 * @author Gregory Tracy
 *
 */
public abstract class StateSelector {
	@SuppressWarnings("rawtypes")
	abstract ArrayList<State> getStates();
}
