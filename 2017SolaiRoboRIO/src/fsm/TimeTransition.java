package fsm;
/**
 * 
 * @author Gregory Tracy
 *
 */
public class TimeTransition extends Transition {
	
	private Long delay;
	
	public TimeTransition(StateSelector stateSelector, State futureState, Long delay) {
		super(stateSelector, futureState);
		this.delay = delay;
	}

	@Override
	public Boolean conditionMet() {
		return this.getTimeAccessor().getTime() >= this.getStateMachine().getCurrentStateLog().getEntranceTime() + this.delay;
	}

}
