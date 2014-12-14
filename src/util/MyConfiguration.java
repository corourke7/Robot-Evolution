package util;

public class MyConfiguration {
	
	// MyChromosome
	// how many myPart we generate in a chromosome
	public static final int partPoolSize = 10;//10
	public static final int partNumOfFeature = 3;
	// how many myJoint we generate in a chromosome
	public static final int maxJoints = partPoolSize-1;//9
	public static final int minJoints = 2;//5
    public static final int jointNumOfFeature = 12;
    public static final float minFriction = 0.01f;
    public static final float maxFriction = 0.99f;
    
    // MyFitnessFunct
    
    // MyJoint
    // for a joint to rotate from min to max in rad
    public static final float maxJointAngle = 2.0f;
    public static final float minJointAngle = 0.0f;
    // max angular velocity for a joint
    public static final float maxAbsoluteAngularVelocity = 0.2f;
    // torque for a joint
	public static final float maxTorque = 70f;//70, 3e10f
	public static final float minTorque = maxTorque;
	// parameter is used for calculating the anchor of a part
    public static final float maxPartParameter = 1.0f;
    public static final float minPartParameter = 0.0f;
    
    // MyPart
    // part size
    public static final float minWidth = 0.5f;
    public static final float maxWidth = 2f;
    public static final float minHeight = 0.5f;
    public static final float maxHeight = 2f;
    
    // MyDemo
    public static final String windowTitle = "Evolution Robot";
    // this is in pixel
    public static final int[] windowDemensions = {1024, 512};
    // 30 pixels = 1 metre
    public static final float pixelsPerMeter = 30f;
    // text position in pixel
    // for drawstring function, the origin is at left top corner.
    public static final int[] textPosition = {20, 20};
    // in demo, we don't make it too fast
	public static final float myDemoTimeStep = 0.01f;//1f/60f;//0.01f;
    // ground length and height in meters
    public static final float[] groundSizeMeter = {200f, 0.5f};
    // we need to kill this robot in finite iterations in demo
 	// Note: this is different from the maxIterationCount in Simulation, because we want to see longer demos
    public static final int maxDemoIterationCount = 1000;
 	// Note: these 2 are different from the maxIterationCount in Simulation, because we want to see longer demos
 	public static final float maxDemoAge = 20f; // 8
    public static final float maxDemoRestAge = 10f;//3
    // evolution
    public static final int populationSize = 50;//100
    public static final int maxEvolutionsPerLoop = 10;
    // every screen we show 20 outputs
    public static final int outputListLength = 20;
    // how thick is one line will take
    public static final int lineHeight = 20;
    
    // main
    
    // MyResult
    
    // Robot
    public static final int robotFilterGroupIndex = -1;
	public static final float robotDensity = 5f;//0.5f;
	public static final float robotFriction = 0.6f;//0.3 between 0 and 1
	// Restitution is used to make objects bounce
	public static final float robotRestitution = 0.5f;//0.5f;
	//public static final float robotAngularDamping = 32f;//0.05f;
	//public static final float robotLinearDamping = 0.5f;
	
	// Simulation
	public static final int velocityIterations = 7;//7
	public static final int positionIterations = 2;//2
	public static final float groundLength = 1000.0f; // in meters
	public static final float gravity = -9.8f;//-9.81f;
	public static final float simuTimeStep = myDemoTimeStep;//1f/60f;
	// Ground position
    // ground length and height in meters
	public static final int groundFilterGroupIndex = 1;
	public static final float groundFriction = 0.6f;//0.3f
	// Restitution is used to make objects bounce
	public static final float groundRestitution = 0.5f;
	// we need to kill this robot in finite iterations in simulation
	// Note: this is different from the maxIterationCount in MyDemo, because we want to see longer demos
	public static final int maxIterationCount = 2000;
	// Note: these 2 are different from the maxIterationCount in MyDemo, because we want to see longer demos
	public static final float maxSimuAge = 20f;
	public static final float maxSimuRestAge = 10f;
    // in Robot.java, we use it to measure the energy use
	public static final float jointMaxPower = (float) (maxTorque*maxAbsoluteAngularVelocity*Math.PI*2);//10f;
	public static final float contactNumberBase = 100f;
    

    
}
