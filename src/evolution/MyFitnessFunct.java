// multithread Java Tutorial http://www.vogella.com/tutorials/JavaConcurrency/article.html

package evolution;
import java.util.ArrayList;

import org.jgap.FitnessFunction;
import org.jgap.Gene;
import org.jgap.IChromosome;

import simulation.MyResult;
import simulation.Simulation;
import util.MyConfiguration;

public class MyFitnessFunct extends FitnessFunction{

	private static final long serialVersionUID = 5328826624969170140L;
	// part pool
	private ArrayList<MyPart> partPool;
	// joint pool
	private ArrayList<MyJoint> jointPool;

	// which part is the body, we have to keep it steady
	private int mainBodyPartIndex;
	private float robotFriction;
	
	private MyResult res;
	
    public double evaluate(IChromosome a_subject){
    	// 1. get information back from the chromosomes
    	retrieveGoodInfo(a_subject);
    	// 2. build the simulation and get the result back
    	Simulation simu = new Simulation(partPool, jointPool, mainBodyPartIndex, robotFriction);
    	res = simu.simulate();
    	//double distance = res.getDistance();
    	//double age = res.getAge();
    	double fitness = res.getFitness();
    	//System.out.println("_____distance = " + distance + "_____age = "+age
    	//		+ "_____fitness = "+ fitness);
    	return fitness;
    }
    
    
	// Parse the current genes out of the chromosome and put the information into arraylists
    // if false, fitness = 0
    public void retrieveGoodInfo(IChromosome a_subject){
    	partPool = new ArrayList<MyPart>();
    	jointPool = new ArrayList<MyJoint>();
    	// 0~29 are the genes of the pool
    	for (int i = 0; i < MyChromosome.partPoolSize; i ++){
 
    		Gene partWidth = a_subject.getGene(i*MyChromosome.partNumOfFeature);
    	    Double tempDouble = new Double((double) partWidth.getAllele());
    		float width = tempDouble.floatValue();
    		
    		Gene partHeight = a_subject.getGene(i*MyChromosome.partNumOfFeature+1);
    	    tempDouble = new Double((double) partHeight.getAllele());
    		float height = tempDouble.floatValue();

    		Gene partNumberOfVertex = a_subject.getGene(i*MyChromosome.partNumOfFeature+2);
    		Boolean numberOfVertex = (Boolean) partNumberOfVertex.getAllele();
	    	// bool True = Triangle, False = Box
    		int tempInt =  (numberOfVertex == true) ? 3 : 4;
    		// currently we do not use triangle
    		//tempInt = 4;
    		
    		
    		MyPart tempPart = new MyPart();
	    	tempPart.setHeight(height);
	    	tempPart.setNumberOfVertex(tempInt);
	    	tempPart.setWidth(width);
	    	partPool.add(tempPart);
    	}

		Gene bodyIndexGene = a_subject.getGene(MyChromosome.partPoolSize*MyChromosome.partNumOfFeature);
		mainBodyPartIndex = new Integer((int) bodyIndexGene.getAllele());
		
		Gene FrictionGene = a_subject.getGene(MyChromosome.partPoolSize*MyChromosome.partNumOfFeature+1);
    	Double tempDouble = new Double((double) FrictionGene.getAllele());
		robotFriction = tempDouble.floatValue();
    	
    	// 31~119
    	for (int i = 0; i < MyChromosome.maxJoints; i ++){
    		int start = MyChromosome.partPoolSize*MyChromosome.partNumOfFeature + 2;
    		start += i * MyChromosome.jointNumOfFeature;
    		Gene jointAngleA = a_subject.getGene(start);
    	    tempDouble = new Double((double) jointAngleA.getAllele());
    		float angleA = tempDouble.floatValue();
    	    assert(angleA<=2*Math.PI && angleA >= 0f);
    		
    		Gene jointAngleBMinusAGene = a_subject.getGene(start+1);
    	    tempDouble = new Double((double) jointAngleBMinusAGene.getAllele());
    		float jointAngleBMinusA = tempDouble.floatValue();
    		float angleB = jointAngleBMinusA + angleA;
    		assert( angleB>= (3*MyConfiguration.minJointAngle-MyConfiguration.maxJointAngle)/2);
    	    assert( angleB<= (3*MyConfiguration.maxJointAngle-MyConfiguration.minJointAngle)/2);
    	    
    		Gene angularVelocityGene = a_subject.getGene(start+2);
    	    tempDouble = new Double((double) angularVelocityGene.getAllele());
    		float angularVelocity = tempDouble.floatValue();
    	    assert(angularVelocity <= MyConfiguration.maxAbsoluteAngularVelocity && angularVelocity >= -MyConfiguration.maxAbsoluteAngularVelocity);
    	    
    		Gene partParameterOne1 = a_subject.getGene(start+3);
    	    tempDouble = new Double((double) partParameterOne1.getAllele());
    		float parameterOne1 = tempDouble.floatValue();

    		Gene partParameterOne2 = a_subject.getGene(start+4);
    	    tempDouble = new Double((double) partParameterOne2.getAllele());
    		float parameterOne2 = tempDouble.floatValue();
    		
    		Gene partParameterTwo1 = a_subject.getGene(start+5);
    	    tempDouble = new Double((double) partParameterTwo1.getAllele());
    		float parameterTwo1 = tempDouble.floatValue();

    		Gene partParameterTwo2 = a_subject.getGene(start+6);
    	    tempDouble = new Double((double) partParameterTwo2.getAllele());
    		float parameterTwo2 = tempDouble.floatValue();

    		Gene partOneIndex = a_subject.getGene(start+7);
    	    int partOne = new Integer((int) partOneIndex.getAllele());
    	    assert(partOne <= MyConfiguration.partPoolSize && partOne >= 0);
    	    
    	    Gene partTwoIndex = a_subject.getGene(start+8);
    	    int partTwo = new Integer((int) partTwoIndex.getAllele());
    	    assert(partTwo <= MyConfiguration.partPoolSize && partTwo >= 0);
    	    
    		Gene useThisJointGene = a_subject.getGene(start+9);
    	    int useThisJoint = new Integer((int) useThisJointGene.getAllele());

    		Gene sequenceGene = a_subject.getGene(start+10);
    		int sequence = new Integer((int) sequenceGene.getAllele());
    		
    		Gene torqueGene = a_subject.getGene(start+11);
    	    tempDouble = new Double((double) torqueGene.getAllele());
    		float torque = tempDouble.floatValue();

    		assert(partOne != partTwo);
    		if (partOne == partTwo) useThisJoint = 0;
    		//assert((partOne != partTwo));
    		
    		// we add it only we use it
	    	if(useThisJoint == 1){
		    	MyJoint tempJoint = new MyJoint();
		    	tempJoint.setJointAngleA(angleA);
		    	tempJoint.setJointAngleB(angleB);
		    	tempJoint.setAngularVelocity(angularVelocity);
		    	tempJoint.setPartParameterOne1(parameterOne1);
		    	tempJoint.setPartParameterOne2(parameterOne2);
		    	tempJoint.setPartParameterTwo1(parameterTwo1);
		    	tempJoint.setPartParameterTwo2(parameterTwo2);
		    	tempJoint.setPartOneIndex(partOne);
		    	tempJoint.setPartTwoIndex(partTwo);
		    	tempJoint.setUseThisJoint(true);
		    	tempJoint.setSequence(sequence);
		    	tempJoint.setTorque(torque);
	    		jointPool.add(tempJoint);
	    	}
    	}
    	//System.out.println("size = " + jointPool.size());
    	assert(jointPool.size()<=MyConfiguration.maxJoints && jointPool.size()>=MyConfiguration.minJoints);
    	assert(partPool.size()==MyConfiguration.partPoolSize);
    	
    	
    }
    public ArrayList<MyPart> getPartPool(){return partPool;}
    public ArrayList<MyJoint> getJointPool(){return jointPool;}
    public int getMainBodyPartIndex(){return mainBodyPartIndex;}
    public float getRobotFriction(){return robotFriction;}
    public MyResult getResult(){return res;}
}
