package fsm;
/**
 * 
 * @author Gregory Tracy
 *
 */
public abstract class Transition {
	private StateSelector stateSelector;
	private State futureState;
	
	private TimeAccessor timeAccessor;
	
	private StateMachine stateMachine;
	
	public Transition(StateSelector stateSelector, State futureState){
		this.stateSelector = stateSelector;
		this.futureState = futureState;
	}
	protected void setStateMachine(StateMachine stateMachine){
		this.stateMachine = stateMachine;
	}
	
	protected void setTimeAccessor(TimeAccessor timeAccessor){
		this.timeAccessor = timeAccessor;
	}
	
	public abstract Boolean conditionMet();
	
	public StateSelector getStateSelector(){
		return this.stateSelector;
	}
	public State getFutureState(){
		return this.futureState;
	}
	
	public StateMachine getStateMachine(){
		return this.stateMachine;
	}
	
	public TimeAccessor getTimeAccessor(){
		return this.timeAccessor;
	}
	
	public void setFutureState(State state){
		this.futureState = state;
	}
}
