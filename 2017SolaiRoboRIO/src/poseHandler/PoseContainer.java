package poseHandler;

/**
 * 
 * @author Gregory Tracy
 *
 * @param <ValueType>
 */
public class PoseContainer<ValueType> {
	
	private PoseHandler handler;
	
	private ValueType[] buffer;
	
	private ValueType currentValue;


	@SuppressWarnings("unchecked")
	public PoseContainer(PoseHandler handler){
		this.handler = handler;
		this.buffer = (ValueType[]) new Object[this.handler.getBufferSize()];
		this.handler.addPoseContainer(this);
	}
	
	protected void addValue(int index){
		this.buffer[index] = this.currentValue;
	}
	
	public synchronized ValueType lookBack(long time){
		
		int index = (this.handler.getCurrentIndex() - 1) - (int) (time/this.handler.getPeriod());
		if(index < 0){
			index = this.handler.getBufferSize() + index;
		}
		return this.buffer[index];
	}
	
	
	
//	GETTERS & SETTERS
	
	public void setCurrentValue(ValueType currentValue) {
		this.currentValue = currentValue;
	}
	
	
	
}
