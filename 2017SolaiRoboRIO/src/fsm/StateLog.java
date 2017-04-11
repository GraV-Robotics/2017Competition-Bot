package fsm;
/**
 * 
 * @author Gregory Tracy
 *
 */
public class StateLog {
	private StateMachine stateMachine;
	private State state;
	
	private Long index;
	
	private Long entranceTime;
	private Long exitTime;
	
	private Long updateCount;
	
	private Transition entranceTransition;
	private Transition exitTransition;
	
	protected StateLog(StateMachine stateMachine, State state, Transition entranceTransition, Long entranceTime, Long index){
		this.stateMachine = stateMachine;
		this.entranceTime = entranceTime;
		this.state = state;
		this.index = index;
		this.updateCount = 0l;
	}

	protected void recordExit(Long time, Transition exitTransition){
		this.exitTime = time;
		this.exitTransition = exitTransition;
	}
	
	protected void incrementUpdateCounter(){
		this.updateCount++;
	}
	
	public State getState(){
		return this.state;
	}
	
	public StateMachine getStateMachine(){
		return this.stateMachine;
	}
	
	public Long getIndex(){
		return this.index;
	}
	
	public Long getEntranceTime(){
		return this.entranceTime;
	}
	
	public Long getExitTime(){
		return this.exitTime;
	}

	public Transition getEntranceTransition(){
		return this.entranceTransition;
	}
	
	public Transition getExitTransition(){
		return this.exitTransition;
	}
	
	public Long getUpdateCount(){
		return this.updateCount;
	}
	
}
