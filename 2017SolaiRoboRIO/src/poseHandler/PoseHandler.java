package poseHandler;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

/**
 * 
 * @author Gregory Tracy
 *
 */
public class PoseHandler extends TimerTask {
	
	private PoseHandlerInterface poseHandlerInterface;
	
	private Integer bufferMemory;
	private Integer period;
	private Integer bufferSize;
	
	private Integer currentIndex;
	
	private Long[] times;
	
	private ArrayList<PoseContainer> poseContainers;
	
	private Timer timer;
	
	
	public PoseHandler(PoseHandlerInterface poseHandlerInterface, int bufferMemory, int period){
		this.poseHandlerInterface = poseHandlerInterface;
		this.bufferMemory = bufferMemory;
		this.period = period;
		this.bufferSize = this.bufferMemory/this.period;
		this.currentIndex = 0;
		this.times = new Long[this.bufferSize];
		
		this.poseContainers = new ArrayList<>();
		
		this.timer = new Timer(true);
	}
	
	public void start(){
		this.timer.schedule(this, 0, this.period);
	}
	
	public void stop(){
		this.timer.cancel();
	}
	
	@Override
	public void run() {
		this.poseHandlerInterface.updatePoseValues();
		this.storePose();
		System.out.println("Hi!");
		System.out.println(this.currentIndex);
	}
	
	public void addPoseContainer(PoseContainer container){
		this.poseContainers.add(container);
	}
	
	private synchronized void incrementIndex(){
		if(this.currentIndex >= this.bufferSize-1){
			this.currentIndex = 0;
		}else{
			this.currentIndex++;
		}
	}
	
	private synchronized void storePose(){
		this.times[this.currentIndex] = this.poseHandlerInterface.getTime();
		for (PoseContainer poseContainer : this.poseContainers) {
			poseContainer.addValue(this.currentIndex);
		}
		this.incrementIndex();
	}
	
//	GETTERS & SETTERS
	
	public int getBufferSize(){
		return this.bufferSize;
	}

	public Integer getCurrentIndex() {
		return this.currentIndex;
	}

	public Integer getPeriod() {
		return this.period;
	}

	
}
