package simulation;

import java.util.ArrayList;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.RevoluteJoint;

import util.MyConfiguration;
import evolution.MyJoint;
import evolution.MyPart;
import gui.MyDemo;

public class Simulation {

	// the world
	public static final int velocityIterations = MyConfiguration.velocityIterations;
	public static final int positionIterations = MyConfiguration.positionIterations;
	public static final float groundLength = MyConfiguration.groundLength; // in meters
	public static final float gravity = MyConfiguration.gravity;
	private World world;
	public static final float timeStep = MyConfiguration.simuTimeStep;
	
	// Ground position
    // ground length and height in meters
    private static final float[] groundSizeMeter = MyConfiguration.groundSizeMeter;
	public static final int groundFilterGroupIndex = MyConfiguration.groundFilterGroupIndex;
	public static final float groundFriction = MyConfiguration.groundFriction;
	// Restitution is used to make objects bounce
	public static final float groundRestitution = MyConfiguration.groundRestitution;
	
	
	private static ArrayList<MyPart> myPartPool;
	private static ArrayList<MyJoint> myJointPool;
//	private static ArrayList<RevoluteJoint> jointPool;
//	private static ArrayList<Body> bodyPool;

	// the robot
	private static Robot myRobot;
	// we need to kill this robot in finite iterations in simulation
	// Note: this is different from the maxIterationCount in MyDemo, because we want to see longer demos
	private static final int maxIterationCount = MyConfiguration.maxIterationCount;
	// currently the max distance the robot have walked
	private static float robotMaxDistance;
	// Note: these 2 are different from the maxIterationCount in MyDemo, because we want to see longer demos
	public static final float maxAge = MyConfiguration.maxSimuAge;
    public static final float maxRestAge = MyConfiguration.maxSimuRestAge;
	// which part is the body, we have to keep it steady
	private int bodyIndex;
    

	public Simulation(ArrayList<MyPart> partPool, ArrayList<MyJoint> jointPool, int bi){
		Simulation.myPartPool = new ArrayList<MyPart>(partPool);
		Simulation.myJointPool = new ArrayList<MyJoint>(jointPool);
//		Simulation.jointPool = new ArrayList<RevoluteJoint>();
//		Simulation.bodyPool = new ArrayList<Body>();
		robotMaxDistance = -1000f;
		this.bodyIndex = bi;
	}
	
	public MyResult simulate(){
		robotMaxDistance = -1000f;
		// the gravity on x and y
        Vec2 gravityVec2 = new Vec2(0f, Simulation.gravity);
        world = new World(gravityVec2);
        world.setAllowSleep(true);
        createGround(this.world);
        myRobot = new Robot(Simulation.myPartPool, Simulation.myJointPool, this.world, this.bodyIndex);
        // get the pools after sort
//        jointPool = myRobot.getJointPool();
        myJointPool = myRobot.getMyJointPool();
        
//        bodyPool = myRobot.getBodyPool();
// Now myJointPool // jointPool // jointSequence
// But partIndexToBody // bodyPool NOT// myPartPool, but all the partIndex are related to myPartPool

        int iterationCount = 0;
        assert(Simulation.maxIterationCount>=100*Simulation.maxAge);
        while(iterationCount <= Simulation.maxIterationCount){
        	iterationCount++;
        	this.world.step(Simulation.timeStep, Simulation.velocityIterations, Simulation.positionIterations);
            motorMoving(myRobot);
        	if(robotDead(myRobot, false) == true)break;
        }
        MyResult mr = myRobot.getResult(this.world);
        myRobot.destroy(this.world);
		return mr;
	}
	
    public static void createGround(World outsideWorld){
    
        BodyDef groundDef = new BodyDef();
        groundDef.position.set(0f, groundSizeMeter[1]/2);
        groundDef.type = BodyType.STATIC;
        PolygonShape shape = new PolygonShape();
        shape.setAsBox(groundSizeMeter[0]/2, groundSizeMeter[1]/2);
        FixtureDef fd = new FixtureDef();
        fd.shape = shape;
        org.jbox2d.dynamics.Filter filter = new org.jbox2d.dynamics.Filter();
        filter.groupIndex = Simulation.groundFilterGroupIndex;
        fd.filter = filter;
        fd.friction = Simulation.groundFriction;
        fd.restitution = Simulation.groundRestitution;
        // create the body and add fixture to it
        Body body = outsideWorld.createBody(groundDef);
        body.createFixture(fd);
    }
    
    public static Boolean robotDead(Robot r, Boolean myDemo){
    	// see how far a robot can walk after each time step which is an iteration of the world
        // if the robot move backward, we reduce its life time
        float distance = r.getDistance();
    	float maxRobotAge = 0f;
    	float maxRobotRestAge = 0f;
    	
    	if (myDemo == true){
    		// if every time the robot can walk forward, then keep going
    		// if it move less than maxDistance, i.e. move back or stay still, 
            // then reduce its life time like natural selection
            if (distance > Simulation.robotMaxDistance){Simulation.robotMaxDistance = distance;} 
            else{r.addRestAge(MyDemo.timeStep);}
            r.addAge(MyDemo.timeStep);
        	
    		maxRobotAge = MyDemo.maxAge;
    		maxRobotRestAge = MyDemo.maxRestAge;
            if(r.isBad(maxRobotAge, maxRobotRestAge, true)) return true;
    	}
    	else{
    		if (distance > Simulation.robotMaxDistance){Simulation.robotMaxDistance = distance;} 
            else{r.addRestAge(Simulation.timeStep);}
            r.addAge(Simulation.timeStep);
        	
    		maxRobotAge = Simulation.maxAge;
    		maxRobotRestAge = Simulation.maxRestAge;
            if(r.isBad(maxRobotAge, maxRobotRestAge, false)) return true;
    	}
    	return false;
    }
    
    
    public static void motorMoving(Robot r){
    	
       	for (int i = 0; i < r.getJointPool().size(); i++) {
    		RevoluteJoint joint = r.getJointPool().get(i);
//    		MyJoint Myjoint = r.getMyJointPool().get(i);
    		// radians per second. Speed can also be negative
    		float speed = joint.getMotorSpeed();
    		assert(Math.abs(joint.getMotorSpeed())/2/Math.PI <= MyJoint.maxAbsoluteAngularVelocity);
  		  	
        	joint.enableLimit(true);
        	joint.enableMotor(true);
        	joint.setMaxMotorTorque(MyConfiguration.maxTorque);
  		  	float curAngle = joint.getJointAngle();
  		  	float lower = joint.getLowerLimit();
  		  	float upper = joint.getUpperLimit();
    		if (curAngle < lower){
    			
//    			System.out.println(i+": <");
//      		  	System.out.print("[up limit= "+(joint.getUpperLimit()));
//    		  	System.out.print(", low limit= "+(joint.getLowerLimit()));
//    		  	System.out.println("], joint angle = "+(joint.getJointAngle()));
//    		  	System.out.println(", joint speed = "+(joint.getMotorSpeed()));
//    			System.out.println("");
    			
    			if(lower-curAngle >= 2*Math.PI - (lower-curAngle) - (upper-lower))
    				joint.setMotorSpeed((float) (-(Math.abs(speed))));
    			else
    				joint.setMotorSpeed((float) ((Math.abs(speed))));
    		}
    		else if(curAngle > upper){

//    			System.out.println(i+": >");
//    			System.out.print("angle > lower - [up limit= "+(joint.getUpperLimit()));
//    		  	System.out.print(", low limit= "+(joint.getLowerLimit()));
//    		  	System.out.println("], joint angle = "+(joint.getJointAngle()));
//    		  	System.out.println(", joint speed = "+(joint.getMotorSpeed()));
    		  	
    			if(curAngle-upper >= 2*Math.PI - (curAngle - upper) - (upper-lower))
    				joint.setMotorSpeed((float) ((Math.abs(speed))));
    			else
    				joint.setMotorSpeed((float) (-(Math.abs(speed))));
    		}
    		else{
//    			System.out.println(i+": ==");		
//      		  	System.out.print("== - [up limit= "+(joint.getUpperLimit()));
//    		  	System.out.print(", low limit= "+(joint.getLowerLimit()));
//    		  	System.out.println("], joint angle = "+(joint.getJointAngle()));
//    		  	System.out.println(", joint speed = "+(joint.getMotorSpeed()));
    			joint.setMotorSpeed(speed);
    		}
		}
    }
}
	
