package org.usfirst.frc.team5816.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
/**
 * 
 * @author Gregory Tracy
 *
 */
public class TegraHandler implements Runnable {
	private DatagramSocket socket;
	private DatagramPacket packet;
	
	private byte[] buffer;
	
	private final int PORT = 5800;
	
	private Double shooterAngle;
	private Boolean shooterIsVisible;
	
	public TegraHandler(){
		this.shooterAngle = 0d;
		this.shooterIsVisible = false;
		Thread core = new Thread(this);
		core.start(); 
	}
	
	public void run(){
		this.shooterIsVisible = false;
		this.shooterAngle = 0d;
		try {
			this.socket = new DatagramSocket(PORT);
			this.buffer = new byte[2048];
			this.packet = new DatagramPacket(this.buffer, this.buffer.length);
		} catch (SocketException e) {
			e.printStackTrace();
		}
		
		while(true){
			String data = null;
			try {
				data = this.receiveData();
			} catch (IOException e) {
				e.printStackTrace();
			}
			String[] tokens = data.split("\n");
			for(String token : tokens){
				this.parseData(token);
			}
			
		}
		
	}
	
	private String receiveData() throws IOException{
		this.socket.receive(this.packet);
		String message = new String(this.buffer, 0, this.packet.getLength());
		this.packet.setLength(this.buffer.length);
		return message;
	}
	
	private void parseData(String data){
		String[] tokens = data.split(":");
//		for(String token : tokens){
//			System.out.println(token);
//		}
		Double arg0 = null;
		try{
			arg0 = Double.parseDouble(tokens[1]);
		}catch(Exception e){
			e.printStackTrace();
			return;
		}
		switch(tokens[0]){
		case "ANGLE":{
			this.shooterAngle = arg0;
			break;
		}
		case "VISIBLE":{
			this.shooterIsVisible = (arg0==1);
			break;
		}
		}
		
	}

	public double getShooterAngle() {
		return shooterAngle;
	}

	public boolean shooterIsVisible() {
		return shooterIsVisible;
	}

}
