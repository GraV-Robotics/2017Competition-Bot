package fsm;
import java.util.ArrayList;
/**
 * 
 * @author Gregory Tracy
 *
 */
public class StateMachine {
	private ArrayList<State> states;
	private ArrayList<Transition> transitions;
	
	private State currentState;
	
	private StateLog[] stateLogs;
	
	private Long stateIndex;
	
	private TimeAccessor timeAccessor;
	
	public StateMachine(TimeAccessor timeAccessor){
		this(timeAccessor, 1000); 
	}
	
	public StateMachine(TimeAccessor timeAccessor, Integer stateLogBuffer){
		this.timeAccessor = timeAccessor;
		this.states = new ArrayList<>();
		this.transitions = new ArrayList<>();
		this.stateLogs = new StateLog[stateLogBuffer];
		this.stateIndex = -1l;
	}
	
	public void addState(State state){
		for (State iterativeState : this.states) {
			if(iterativeState == state){
				return;
			}
		}
		this.states.add(state);
		state.setTimeAccessor(this.timeAccessor);
		state.setStateMachine(this);
	}
	
	public void removeState(State state){
		this.exitState(null);
		this.currentState = null;
		this.states.remove(state);
	}
	
	public void clearStates(State state){
		this.exitState(null);
		this.currentState = null;
		this.states.clear();
	}
	
	public void addTransition(Transition transition){
		for (Transition iterativeTransition : this.transitions) {
			if(iterativeTransition == transition){
				return;
			}
		}
		this.transitions.add(transition);
		transition.setTimeAccessor(this.timeAccessor);
		transition.setStateMachine(this);
	}
	
	public void removeTransition(Transition transition){
		this.exitState(null);
		this.currentState = null;
		this.transitions.remove(transition);
	}
	
	public void clearTransitions(){
		this.exitState(null);
		this.currentState = null;
		this.exitState(null);
	}
	
	public ArrayList<State> getStates(){
		return this.states;
	}
	
	public StateLog getCurrentStateLog(){
		return this.getPreviousStateLog(0);
	}
	
	private void setCurrentStateLog(StateLog stateLog){
		this.stateLogs[this.getLogIndex(this.stateIndex)] = stateLog;
	}
	
	public StateLog getPreviousStateLog(Integer index){
		if(this.stateIndex - index < 0){
			return null;
		}
		return this.stateLogs[this.getLogIndex(this.stateIndex - index)];
	}
	
	public Long getStateIndex(){
		return this.stateIndex;
	}
	
	protected Integer getLogIndex(Long index){
		return (int) (index % this.stateLogs.length);
	}
	
	protected void enterState(State state, Transition transition){
		this.stateIndex++;
		this.currentState = state;
		this.setCurrentStateLog(new StateLog(this, state, transition, this.timeAccessor.getTime(), this.stateIndex));
		state.enter();
	}
	
	protected void exitState(Transition transition){
		if(this.currentState == null){
			return;
		}
		this.currentState.exit();
		this.getCurrentStateLog().recordExit(this.timeAccessor.getTime(), transition);
	}
	
	public void exitState(){
		this.exitState(null);
	}
	
	synchronized protected void setState(State state, Transition transition){
		if(this.currentState != null){
			this.exitState(transition);
		}
		this.enterState(state, transition);
	}
	
	synchronized public void setState(State state){
		this.setState(state, null);
	}
	
	protected ArrayList<Transition> getStateTransitions(State state){
		ArrayList<Transition> output = new ArrayList<>();
		for(Transition transition : this.transitions){
			for(State iterativeState : transition.getStateSelector().getStates()){
				if(state == iterativeState){
					output.add(transition);
					break;
				}
			}
		}
		return output;
	}
	
	synchronized public void update(){
		
		this.currentState.update();
		for(Transition transition : this.getStateTransitions(this.currentState)){
			if(transition.conditionMet()){
				this.setState(transition.getFutureState(), transition);
				break;
			}
		}
	}
	
	public void reset(){
		this.exitState(null);
		this.currentState = null;
		this.stateIndex = -1l;
		for(int index = 0; index < this.stateLogs.length; index++){
			this.stateLogs[index] = null; 
		}
	}
	
}
