package fsm;
/**
 * 
 * @author Gregory Tracy
 *
 */
public abstract class State {
	
	private Long minimumUpdateInterval;
	private Long lastUpdateTime;
	
	private TimeAccessor timeAccessor;
	
	private StateMachine stateMachine;
	

	public State(){
		this(0l);
	}
	
	public State(Long minimumUpdateInterval){
		this.minimumUpdateInterval = minimumUpdateInterval;
	}
	protected void setStateMachine(StateMachine stateMachine){
		this.stateMachine = stateMachine;
	}
	
	protected void setTimeAccessor(TimeAccessor timeAccessor){
		this.timeAccessor = timeAccessor;
	}
	
	protected abstract void enter();
	
	protected abstract void periodic();
	
	protected abstract void exit();
	
	protected void update(){
		if(this.lastUpdateTime == null || this.timeAccessor.getTime() >= this.lastUpdateTime + this.minimumUpdateInterval){
			this.lastUpdateTime = this.timeAccessor.getTime();
			this.stateMachine.getCurrentStateLog().incrementUpdateCounter();
			this.periodic();
		}
	}
	
	public StateMachine getStateMachine(){
		return this.stateMachine;
	}
	
	
	

	

	
}
