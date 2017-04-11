package fsm;

import java.util.ArrayList;

/**
 * 
 * @author Gregory Tracy
 *
 */
public class AllStatesSelector extends StateSelector {
	private StateMachine stateMachine;
	public AllStatesSelector(StateMachine stateMachine){
		this.stateMachine = stateMachine;
	}
	
	@Override
	ArrayList<State> getStates() {
		return this.stateMachine.getStates();
	}
}
