package simulation;
/* Basic program structure is based on corazza/Robot-Evolution Github 
* https://github.com/corazza/Robot-Evolution
* @author Shen Li, Colleen O'Rourke
*/
import java.util.ArrayList;

import evolution.MyJoint;
import evolution.MyPart;


public class MyResult implements Comparable<MyResult> {

	// part pool
//	private ArrayList<MyPart> myPartPool;
	// joint pool
//	private ArrayList<MyJoint> myJointPool;

	private double distance;
	private double age;
	private double fitness;
	private double totalEnergyUse;
	
    public MyResult() {
    	this.distance = 0.0;
    	this.age = 0.0;
    	this.fitness = 0.0;
    	this.totalEnergyUse = 0.0;
//    	this.myPartPool = new ArrayList<MyPart>();
//    	this.myJointPool = new ArrayList<MyJoint>();
    }
    
    public MyResult(ArrayList<MyPart> partPool, ArrayList<MyJoint> myJointPool, 
    		float distance, float age, float totalEnergyUseSource,  float fitness){
    	this.distance = distance >= 0 ? distance : 0;
    	this.age = age >= 0 ? age : 0;
    	this.fitness = fitness >= 0 ? fitness : 0;
    	this.totalEnergyUse = totalEnergyUseSource >= 0 ? totalEnergyUseSource : 0;
//    	this.myPartPool = new ArrayList<MyPart>(partPool);
//    	this.myJointPool = new ArrayList<MyJoint>(myJointPool);
    }

    public int compareTo(MyResult other){
    	double difference = this.fitness - other.fitness;
        if (difference < 0) {return -1;} 
        else if (difference > 0) {return 1;} 
        else{return 0;}
    }

    public double getFitness(){return fitness;}
    public double getAge(){return age;}
    public double getDistance(){return distance;}
    public double getEnergyUsePerJoint(){return totalEnergyUse;}
    
    @Override
    public String toString() {
    	return ("Distance = " + distance + "_____age = "+age + "_____fitness = "+ fitness);
    }

}

