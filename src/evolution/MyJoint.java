package evolution;
/* Basic program structure is based on corazza/Robot-Evolution Github 
* https://github.com/corazza/Robot-Evolution
* @author Shen Li, Colleen O'Rourke
*/
import java.util.ArrayList;
import java.util.List;

import util.MyConfiguration;

// Joint connects 2 parts of robot
public class MyJoint {

	// we need the joint to rotate from A to B
    public static final float maxJointAngle = MyConfiguration.maxJointAngle;
    public static final float minJointAngle = MyConfiguration.minJointAngle;
    private float jointAngleA;
    private float jointAngleB;
    
    // angular velocity for a joint
    public static final float maxAbsoluteAngularVelocity = MyConfiguration.maxAbsoluteAngularVelocity;
    private float angularVelocity;
    
    private float torque;
	public static final float maxTorque = MyConfiguration.maxTorque;
	public static final float minTorque = MyConfiguration.minTorque;
    
	// parameter is used for calculating the anchor of a part
    public static final float maxPartParameter = MyConfiguration.maxPartParameter;
    public static final float minPartParameter = MyConfiguration.minPartParameter;
	private float partParameterOne1;
	private float partParameterOne2;
	private float partParameterTwo1;
	private float partParameterTwo2;

	// the 2 parts which this joint is connecting
	private MyPart partOne;
	private MyPart partTwo;

    // the index in the part pool in a chromosome
	private int partOneIndex;
	private int partTwoIndex;
    
    // the indicator that if we use this joint or not
	private Boolean useThisJoint;
	
	// the sequence of joint rotation
	private int sequence;	
	
	
    public float getJointAngleA(){return this.jointAngleA;}
    public float getJointAngleB(){return this.jointAngleB;}
    public float getAngularVelocity(){return this.angularVelocity;}
    public float getPartParameterOne1(){return this.partParameterOne1;}
    public float getPartParameterOne2(){return this.partParameterOne2;}
    public float getPartParameterTwo1(){return this.partParameterTwo1;}
    public float getPartParameterTwo2(){return this.partParameterTwo2;}
    public int getPartOneIndex(){return this.partOneIndex;}
    public int getPartTwoIndex(){return this.partTwoIndex;}
    public int getSequece(){return this.sequence;}
    public Boolean getUseThisJoint(){return this.useThisJoint;}
    // get 2 parts in a list
    public List<MyPart> getPartList() {
        ArrayList<MyPart> List = new ArrayList<MyPart>();
        List.add(partOne);
        List.add(partTwo);
        return List;
    }
    public float getTorque(){return this.torque;}
    
    public void setJointAngleA(float source){this.jointAngleA = source;}
    public void setJointAngleB(float source){this.jointAngleB = source;}
    public void setPartParameterOne1(float source){this.partParameterOne1 = source;}
    public void setPartParameterOne2(float source){this.partParameterOne2 = source;}
    public void setPartParameterTwo1(float source){this.partParameterTwo1 = source;}
    public void setPartParameterTwo2(float source){this.partParameterTwo2 = source;}
    public void setUseThisJoint(Boolean source){this.useThisJoint = source;}
    public void setPartOneIndex(int source){
    	if (source >= MyChromosome.partPoolSize || source < 0)
    		throw new IllegalArgumentException("ERROR in Joint.java, setPartOneIndex");
    	else this.partOneIndex = source;}
    public void setPartTwoIndex(int source){
    	if (source >= MyChromosome.partPoolSize || source < 0)
    		throw new IllegalArgumentException("ERROR in Joint.java, setPartTwoIndex");
    	else this.partTwoIndex = source;}
    public void setAngularVelocity(float source){
        if (source > maxAbsoluteAngularVelocity) this.angularVelocity = maxAbsoluteAngularVelocity;
        else if (source < -maxAbsoluteAngularVelocity) this.angularVelocity = -maxAbsoluteAngularVelocity;
        else this.angularVelocity = source;
    }
    public void setSequence(int source){
    	if(source > MyChromosome.maxJoints) this.sequence = MyChromosome.maxJoints;
    	else if(source <= 0) this.sequence = 0;
    	else this.sequence = source;
    }
    public void setTorque(float source){this.torque = source;}
    
    @Override
    public String toString() {
        return "PartJoint [jointAngleA=" + jointAngleA + ", jointAngleB=" + jointAngleB
                + ", angularVelocity=" + angularVelocity + ", partParameterOne1="
                + partParameterOne1 + ", partParameterOne2=" + partParameterOne2 
                + "partParameterTwo1="+partParameterTwo1+ ", partParameterTwo2=" + partParameterTwo2 
                + ", partOne=" + partOne + ", partTwo=" + partTwo + "]";
    }
    
}
















