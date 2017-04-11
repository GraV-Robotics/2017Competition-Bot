package fsm;
/**
 * 
 * @author Gregory Tracy
 *
 */
public class CountTransition extends Transition {

	private Long count;
	
	public CountTransition(StateSelector stateSelector, State futureState, Long count) {
		super(stateSelector, futureState);
		this.count = count;
	}

	@Override
	public Boolean conditionMet() {
		return this.getStateMachine().getCurrentStateLog().getUpdateCount() >= this.count;
	}

}
