package evolution;

import java.util.ArrayList;

import org.jgap.Configuration;
import org.jgap.Gene;
import org.jgap.InvalidConfigurationException;
import org.jgap.impl.BooleanGene;
import org.jgap.impl.DoubleGene;
import org.jgap.impl.IntegerGene;

import util.MyConfiguration;

// this is the chromosome ~ a robot. This includes all the parts and joints in 1 robot
public class MyChromosome {

	/*
	 * for each chromosome, the part pool has 10 parts
	 * 						the joint pool has 10 joints
	 * for each joint, there are 2 parts from the part pool
	 * Now let's say now we are using 5 joints from 7 parts
	 * 1. Evolve the 10 parts' features
	 * 2. Evolve the 10 joints' features
	 * 3. Randomly choose n joints from the pool, n should be in [1,10]
	 * 		Randomly choose 2 parts from the pool for this joint
	 * 4. update the partJoints for each part
	 * 
	 */
	// parts
	public static final int partPoolSize = MyConfiguration.partPoolSize;
	public static final int partNumOfFeature = MyConfiguration.partNumOfFeature;

	// joints
	public static final int maxJoints = MyConfiguration.maxJoints;
	public static final int minJoints = MyConfiguration.minJoints;
    public static final int jointNumOfFeature = MyConfiguration.jointNumOfFeature;
	
	
    public MyChromosome(){}
    
    public static ArrayList<Gene> generateGenes(Configuration conf){
    	//System.out.println(conf);
    	//System.out.println();
	    /*
	   	 * 1. Evolve the 10 parts' features
	   	 * 2. Evolve the 5 joints' features
	   	 * 3. Randomly choose n joints from the pool, n should be in [1,10]
	   	 * 		Randomly choose 2 parts from the pool for this joint
	   	 * 4. update the partJoints for each part
	   	 * 
	   	 */
    	try{
	    	ArrayList<Gene> genes = new ArrayList<Gene>();
	    	// 1. Evolve the 10 parts' features
	    	//    partPoolGene = 10 * onePart = 10 * (3 features) = 30 elements
	    	for (int i = 0; i < MyChromosome.partPoolSize; i++){
		    	Gene partWidth = new DoubleGene(conf, MyPart.minWidth, MyPart.maxWidth);
		    	Gene partHeight = new DoubleGene(conf, MyPart.minHeight, MyPart.maxHeight);
		    	// bool True = Triangle, False = Box
		    	Gene partNumberOfVertex = new BooleanGene(conf);
		    	genes.add(partWidth);
		    	genes.add(partHeight);
		    	genes.add(partNumberOfVertex);
		    }

	    	// which part is the body, we have to keep it steady
	    	Gene mainBodyPartIndex = new IntegerGene(conf, 0, partPoolSize-1);
	    	genes.add(mainBodyPartIndex);
	    	
		   	// 2. Evolve the 12 joints' features and Randomly choose n joints from the pool, n should be in [1,10]
	    	//    jointPoolGene = 10 * oneJoint = 10 * (12 features) = 120 elements
	    	assert(MyChromosome.partPoolSize - MyChromosome.maxJoints >= 1);
    		for (int i = 0; i < MyChromosome.minJoints; i++){
		    	
	    		/* to prevent the Joint connecting 12, 23, 31 become a cycle and behave strangely
	    		 * we have to prevent cycles:
	    		 * 	we first create joint A to connect 1,2
	    		 *  then create joint B to connect a new body with 1 or 2
	    		 *  ...
	    		 *  http://www.box2d.org/forum/viewtopic.php?f=3&t=8859
	    		 *  
	    		 *  for example, the 1st joint (i = 0) will connect to the body 0 and 1,
	    		 *  the 2nd joint (i = 1) will connect to the body 2 and another one from 0 or 1
	    		 *  the 3rd joint (i = 2) will connect to the body 3 and another one from 0, 1, or 2
	    		 */
	    		assert(i <= MyChromosome.partPoolSize - 1);
	    		
	    		Gene jointAngleA = new DoubleGene(conf, MyJoint.minJointAngle, MyJoint.maxJointAngle);
		    	float diff = (MyJoint.maxJointAngle - MyJoint.minJointAngle)/2;
		    	Gene jointAngleBMinusA = new DoubleGene(conf, -1*diff, diff);
		    	Gene angularVelocity = new DoubleGene(conf, -MyJoint.maxAbsoluteAngularVelocity, MyJoint.maxAbsoluteAngularVelocity);
		    	Gene partParameterOne1 = new DoubleGene(conf, MyJoint.minPartParameter, MyJoint.maxPartParameter);
		    	Gene partParameterOne2 = new DoubleGene(conf, MyJoint.minPartParameter, MyJoint.maxPartParameter);
		    	Gene partParameterTwo1 = new DoubleGene(conf, MyJoint.minPartParameter, MyJoint.maxPartParameter);
		    	Gene partParameterTwo2 = new DoubleGene(conf, MyJoint.minPartParameter, MyJoint.maxPartParameter);
		    	
		    	Gene partOneIndex;
		    	Gene partTwoIndex;
		    	if(i==0){
			    	partOneIndex = new IntegerGene(conf, 0, 0);
			    	partTwoIndex = new IntegerGene(conf, 1, 1);
		    	}
		    	else{
		    		partOneIndex = new IntegerGene(conf, i+1, i+1);
			    	partTwoIndex = new IntegerGene(conf, 0, i);
		    	}
		    	
		    	// we must use these joints to keep min number
		    	Gene useThisJoint = new IntegerGene(conf, 1, 1);
		    	// we also care about the sequence of the joint rotating
		    	Gene sequence = new IntegerGene(conf, 0, MyChromosome.maxJoints);
		    	Gene torque = new DoubleGene(conf, MyJoint.minTorque, MyJoint.maxTorque);
		    	
		    	
		    	genes.add(jointAngleA);
		    	genes.add(jointAngleBMinusA);
		    	genes.add(angularVelocity);
		    	genes.add(partParameterOne1);
		    	genes.add(partParameterOne2);
		    	genes.add(partParameterTwo1);
		    	genes.add(partParameterTwo2);
		    	genes.add(partOneIndex);
		    	genes.add(partTwoIndex);
		    	genes.add(useThisJoint);
		    	genes.add(sequence);
		    	genes.add(torque);
		    }
	    	for (int i = MyChromosome.minJoints; i < MyChromosome.maxJoints; i++){
	    		assert(i <= MyChromosome.partPoolSize - 1);
	    		// in fitness function, if part 1 = 2, we have to discard this joint
		    	Gene jointAngleA = new DoubleGene(conf, MyJoint.minJointAngle, MyJoint.maxJointAngle);
		    	float diff = (MyJoint.maxJointAngle - MyJoint.minJointAngle)/2;
		    	Gene jointAngleBMinusA = new DoubleGene(conf, -1*diff, diff);
		    	Gene angularVelocity = new DoubleGene(conf, -MyJoint.maxAbsoluteAngularVelocity, MyJoint.maxAbsoluteAngularVelocity);
		    	Gene partParameterOne1 = new DoubleGene(conf, MyJoint.minPartParameter, MyJoint.maxPartParameter);
		    	Gene partParameterOne2 = new DoubleGene(conf, MyJoint.minPartParameter, MyJoint.maxPartParameter);
		    	Gene partParameterTwo1 = new DoubleGene(conf, MyJoint.minPartParameter, MyJoint.maxPartParameter);
		    	Gene partParameterTwo2 = new DoubleGene(conf, MyJoint.minPartParameter, MyJoint.maxPartParameter);
		    	
		    	Gene partOneIndex;
		    	Gene partTwoIndex;
		    	if(i==0){
			    	partOneIndex = new IntegerGene(conf, 0, 0);
			    	partTwoIndex = new IntegerGene(conf, 1, 1);
		    	}
		    	else{
		    		partOneIndex = new IntegerGene(conf, i+1, i+1);
			    	partTwoIndex = new IntegerGene(conf, 0, i);
		    	}
		    	// we can add this joint to the pool and also choose if we use this joint or not
		    	Gene useThisJoint = new IntegerGene(conf, 0, 1);
		    	// we also care about the sequence of the joint rotating
		    	Gene sequence = new IntegerGene(conf, 0, MyChromosome.maxJoints);
		    	Gene torque = new DoubleGene(conf, MyJoint.minTorque, MyJoint.maxTorque);
		    	
		    	genes.add(jointAngleA);
		    	genes.add(jointAngleBMinusA);
		    	genes.add(angularVelocity);
		    	genes.add(partParameterOne1);
		    	genes.add(partParameterOne2);
		    	genes.add(partParameterTwo1);
		    	genes.add(partParameterTwo2);
		    	genes.add(partOneIndex);
		    	genes.add(partTwoIndex);
		    	genes.add(useThisJoint);
		    	genes.add(sequence);
		    	genes.add(torque);
		    }
	    	return genes;
    	}
    	catch(InvalidConfigurationException i){
    		System.out.println("ERROR generateGenes - InvalidConfigurationException");
    	}
		return null;
    }
}
