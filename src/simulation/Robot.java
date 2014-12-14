package simulation;

import java.util.ArrayList;
import java.util.HashMap;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;

import util.JointSeq;
import util.MyConfiguration;
import evolution.MyChromosome;
import evolution.MyJoint;
import evolution.MyPart;

//1 part ~ 1 Body, 1 Robot ~ 1 Chromosome, 1 simulation ~ 1 world
public class Robot {

	public static final int robotFilterGroupIndex = MyConfiguration.robotFilterGroupIndex;
	public static final float robotDensity = MyConfiguration.robotDensity;//0.5f;
	public static final float robotFriction = MyConfiguration.robotFriction;//0.3
	// Restitution is used to make objects bounce
	public static final float robotRestitution = MyConfiguration.robotRestitution;
	//public static final float robotAngularDamping = MyConfiguration.robotAngularDamping;
	//public static final float robotLinearDamping = MyConfiguration.robotLinearDamping;

    // we use it to limit a robot's running time
    private float age;
	// how long the robot has been resting, not moving forwards
    private float restAge;
    
	// Body
	private static float bodyX = 4f; //8f;
//	private static float bodyY; //2f;	

	// which part is the body, we have to keep it steady
	private static int mainBodyPartIndex;
	private static Body mainBody;
	
	
	// part pool
	private ArrayList<MyPart> myPartPool;
	//  Body pool in JBox2D
	private ArrayList<Body> bodyPool;
	// joint pool
	private ArrayList<MyJoint> myJointPool;
	// joint pool in JBox2D
	private ArrayList<RevoluteJoint> jointPool;
	// we use this list to keep track of which parts have been created and which not
	// partCreated should be parallel to bodyPool
    private HashMap<Integer, Body> partIndexToBody;
    
    private ArrayList<Integer> jointSequence;
    
	// calculate the fitness
	private Float fitness;
    
	// the coordinate in last timestep
	private static float prevX;
//	private static float prevY;
	
	// the power of the robot when moving in the whole time = torque * angular velocity
	// so the total work the robot will do is the integral of power on time
	// torque = joule per radian
	private float totalEnergyUse;
	private float totalEnergyUseBase;
	// torque * angular velocity = power
	private static final float jointMaxPower = MyConfiguration.jointMaxPower;
	
	// we want it to move in constant speed, not just roll over and stop
	private ArrayList<Float> velocities;
	private float totalVelocity;
	private int numberOfVelocity;
	
	
	public Robot(ArrayList<MyPart> partPoolSource, ArrayList<MyJoint> jointPoolSource, 
			World world, int mainBodyIndex){
		
		this.bodyPool = new ArrayList<Body>();
		this.myPartPool = new ArrayList<MyPart>(partPoolSource);
		this.myJointPool = new ArrayList<MyJoint>(jointPoolSource);
		this.jointPool = new ArrayList<RevoluteJoint>();
		this.partIndexToBody = new HashMap<Integer, Body>();
		this.jointSequence = new ArrayList<Integer>();
		this.fitness = 0f;
		Robot.prevX = 0f;
//		Robot.prevY = 0f;
		Robot.mainBodyPartIndex = mainBodyIndex;
		assert(mainBodyIndex>=0 && mainBodyIndex<MyChromosome.partPoolSize);
		totalEnergyUse = 0.0f;
		totalEnergyUseBase = 0.0f;
		this.velocities = new ArrayList<Float>();
		totalVelocity = 0f;
		numberOfVelocity = 0;
		
		// for each joint, generate 2 Body for 2 parts and put the joint in jointPool
		// But we do not want to generate the same part twice, so we use partToBody to keep track
        for (MyJoint tempJoint : this.myJointPool) {
            // if the bodies for the two parts have not been created, then create them
        	int partOneIndex = tempJoint.getPartOneIndex();
            if(this.partIndexToBody.containsKey(partOneIndex) == false){
            	// create the body and put it in the bodyPool in JBox2D
            	Body tempBody = createBody(this.myPartPool.get(partOneIndex), world);
            	//this.myPartPool.get(partOneIndex).testBody = tempBody;
            	// look for the mainBody
            	if (mainBodyPartIndex == partOneIndex){
            		tempBody.m_angularVelocity = 0;
            		mainBody = tempBody;
            	}
            	this.bodyPool.add(tempBody);
            	this.partIndexToBody.put(partOneIndex, tempBody);
            }
        	int partTwoIndex = tempJoint.getPartTwoIndex();
            if(this.partIndexToBody.containsKey(partTwoIndex) == false){
            	// create the body and put it in the bodyPool in JBox2D
            	Body tempBody = createBody(this.myPartPool.get(partTwoIndex), world);
            	//this.myPartPool.get(partTwoIndex).testBody = tempBody;
            	// look for the mainBody
            	if (mainBodyPartIndex == partTwoIndex){
            		tempBody.m_angularVelocity = 0;
            		mainBody = tempBody;
            	}
            	this.bodyPool.add(tempBody);
            	this.partIndexToBody.put(tempJoint.getPartTwoIndex(), tempBody);
            }
            // after these, bodyPool's order is messed, But myPartPool is still in original order
            // create a joint for each joint and add it to jointPool in JBox2D

            // jointPool and myJointPool are parallel
            this.jointPool.add(createJoint(tempJoint, world));
            this.jointSequence.add(tempJoint.getSequece());
        }
        
        // distribute an existing body if previously not
        if(mainBody == null){mainBody = bodyPool.get(mainBodyPartIndex % bodyPool.size());}
        
        // sort the lists according to the sequence
// Now myJointPool // jointPool // jointSequence
// But partIndexToBody // bodyPool NOT// myPartPool, but all the partIndex are related to myPartPool
        JointSeq st = new JointSeq(myJointPool, jointPool, jointSequence);
        //Joint a = jointPool.get(0);
        //int b = jointSequence.get(0);
        //MyJoint c = myJointPool.get(0);
        st.sort();
        myJointPool = new ArrayList<MyJoint>(st.getMyJointPool());
        jointPool = new ArrayList<RevoluteJoint>(st.getJointPool());
        jointSequence = new ArrayList<Integer>(st.getSequence());
        /*
        assert(myJointPool.size() == jointPool.size() && jointPool.size() == jointSequence.size());
        for(int i = 0; i < jointPool.size(); i++){
        	if(jointPool.get(i).equals(a)){
        		assert(myJointPool.get(i).equals(c) && jointSequence.get(i).equals(b));
        	}
        }*/
	}
	
	
	// generate a body for each part
    private Body createBody(MyPart part, World world){
    	float width = part.getWidth();
    	float height = part.getHeight();
    	int numberOfVertex = part.getNumberOfVertex();
    	
        // body definition
        BodyDef bd = new BodyDef();
		// starting position - this is the position of the center of the Body
        //bd.position.set(2f, 10f);
        bd.position.set(Robot.bodyX, height/2);
        bd.angle = 0f;
        bd.angularVelocity = 0f;
        // reduce bounce
        //bd.angularDamping = MyConfiguration.robotAngularDamping;
        //bd.linearDamping = Robot.robotLinearDamping;
        bd.type = BodyType.DYNAMIC;
        // define shape of the body.
        PolygonShape Shape = new PolygonShape();
        // this is all in local coordinate
        if (numberOfVertex == 4){
        	Vec2 vertices[] = new Vec2[4];
        	vertices[0] = new Vec2(0f, 0f);
        	vertices[1] = new Vec2(width, 0f);
        	vertices[2] = new Vec2(width, height);
        	vertices[3] = new Vec2(0, height);
        	int count = 4;
        	Shape.set(vertices, count);
        }
        else if (numberOfVertex == 3){
        	float side = Math.min(height, width);
        	Vec2 vertices[] = new Vec2[3];
        	vertices[0] = new Vec2(0f, 0f);
        	vertices[1] = new Vec2(side, 0f);
        	vertices[2] = new Vec2(side/2, side/2f*1.732f);
        	int count = 3;
        	Shape.set(vertices, count);
        }
        else throw new IllegalArgumentException("ERROR in Robot.java line 77 - numberOfVertex");

        // define fixture of the body.
        FixtureDef fd = new FixtureDef();
        // the bodies in the same filter group will not collide
        org.jbox2d.dynamics.Filter filter = new org.jbox2d.dynamics.Filter();
        filter.groupIndex = Robot.robotFilterGroupIndex;
        fd.filter = filter;
        fd.shape = Shape;
        fd.density = Robot.robotDensity;
        fd.friction = Robot.robotFriction;
        fd.restitution = Robot.robotRestitution; 
        // create the body and add fixture to it
        Body body = world.createBody(bd);
        body.createFixture(fd);
        return body;
    }
	
	/* The revolute joint can be thought of as a hinge, a pin, or an axle. 
	 * An anchor point is defined on each body, and the bodies will be moved so that 
	 * these two points are always in the same place, and the relative rotation of the bodies 
	 * is not restricted.
	 * 
	 * Revolute joints can be given limits so that the bodies can rotate only to a certain point. 
	 * They can also be given a motor so that the bodies will try to rotate at a given speed, 
	 * with a given torque. Common uses for revolute joints include:
	
	 */
    // every MyJoint will be a Joint to connect 2 Body.
    private RevoluteJoint createJoint(MyJoint joint, World world){
    	int oneIndex = joint.getPartOneIndex();
    	int twoIndex = joint.getPartTwoIndex();
    	MyPart myPartOne = myPartPool.get(oneIndex);
    	MyPart myPartTwo = myPartPool.get(twoIndex);
        Body bodyOne = partIndexToBody.get(oneIndex);
        Body bodyTwo = partIndexToBody.get(twoIndex);
        //assert(bodyOne.equals(myPartOne.testBody));
        //assert(bodyTwo.equals(myPartTwo.testBody));
        
		// http://www.iforce2d.net/b2dtut/joints-revolute
		// The revolute joint can be thought of as a hinge, a pin, or an axle. 
		// An anchor point is defined on each body, and the bodies will be moved 
		// so that these two points are always in the same place, and the relative rotation of the bodies is not restricted.
        RevoluteJointDef jointDef = new RevoluteJointDef();
        jointDef.bodyA = bodyOne;
        jointDef.bodyB = bodyTwo;

		// http://www.iforce2d.net/b2dtut/joints-revolute
		// The 2 bodies will rotate around their respective anchors. 
        // Their bodies are intersected at the anchor point but their fixtures are not intersected.
        
        /* The anchor position of each body is in local body coordinates
         * Placing each Body on the origin, and choose the anchor point
         * But in fact, the Body maybe placed on other position, like (0,-5)
         * anchor ~ Body in local coordinates = anchor ~ Body in world coordinates
         */
		// So the 2 bodies can rotate around the joint position
        jointDef.localAnchorA = myPartOne.getAnchor(joint.getPartParameterOne1(), joint.getPartParameterOne2());
        jointDef.localAnchorB = myPartTwo.getAnchor(joint.getPartParameterTwo1(), joint.getPartParameterTwo2());
        /*Note that the joint angle increases as bodyB rotates counter-clockwise in relation to bodyA. 
         * If both bodies were moved or rotated in the same way, the joint angle does not change 
         * because it represents the relative angle between the bodies
         */
        jointDef.lowerAngle = Math.min(joint.getJointAngleA(), joint.getJointAngleB());
        jointDef.upperAngle = Math.max(joint.getJointAngleA(), joint.getJointAngleB());
		
        // The maximum motor torque (moment) used to achieve the desired motor speed, in Joules
        // initial max torque is really small compared to the normal one
        jointDef.maxMotorTorque = MyConfiguration.maxTorque;
        // motorSpeed: The DESIRED motor speed, radians per second.
        jointDef.motorSpeed = (float)(joint.getAngularVelocity()*2.0f*Math.PI);
		// enableMotor: A flag to enable the joint motor
        jointDef.enableMotor = true;
		// limit it between lowerAngle and upperAngle, if beyond, then rotate back to this range
        jointDef.enableLimit = true;
        return (RevoluteJoint) world.createJoint(jointDef);
    }
    
    
    
    // every time we add age we calculate the fitness
    // "Applying Neural Networks to Control Gait of Simulated Robots" by Heinen and Os¨®rio
    public void addAge(float curTimeStep){
    	// A. add age
    	this.age += curTimeStep;
    	// B. calculate the fitness during this timestep
		// 1. we should calculate the variance of each part so that it does not fall apart
    	float totalX = 0f;
    	float totalY = 0f;
    	for (Body body : this.bodyPool){
    		float x = body.getPosition().x;
    		float y = body.getPosition().y;
    		totalX += x;
    		totalY += y;
    	}
    	float aveX = totalX / this.bodyPool.size();
    	float aveY = totalY / this.bodyPool.size();
    	float totalSRX = 0f;
    	float totalSRY = 0f;
    	for (Body body : this.bodyPool){
    		float x = body.getPosition().x;
    		float y = body.getPosition().y;
    		totalSRX += (x-aveX)*(x-aveX);
    		totalSRY += (y-aveY)*(y-aveY);
    	}
    	float bodyVariance = (float) Math.sqrt((totalSRX + totalSRY)/this.bodyPool.size());

    	// 2. how far does it walk in this timestep
    	float distance = this.getDistance() - Robot.prevX;

    	// 3. velocity variance to keep it in constant speed
    	float v = distance/MyConfiguration.simuTimeStep;
    	velocities.add(v);
    	totalVelocity += v;
    	numberOfVelocity++;
    	
    	
    	// 4. calculate the power
    	// http://en.wikipedia.org/wiki/Torque#Relationship_between_torque.2C_power.2C_and_energy
    	// moving of Bodies are free rotation, Only the movements caused by motor will consume energy
    	// more power consuming - worse
    	float energyUse = 0f;
    	// we have to use joint to calculate the energy because we have no torque on Bodies
    	for(RevoluteJoint j : jointPool){
    		energyUse += Math.abs(j.getMaxMotorTorque()*j.getMotorSpeed());
	    	/*System.out.println("torque= "+j.getMaxMotorTorque());
	    	System.out.println("speed= "+j.getMotorSpeed());
	    	System.out.println("e= "+Math.abs(j.getMaxMotorTorque()*j.getMotorSpeed()));*/
    	}
    	totalEnergyUse += energyUse;
    	totalEnergyUseBase += jointMaxPower * jointPool.size();
    	//System.out.println("energyUse = " + energyUse);
    	
    	// the total fitness
    	this.fitness += distance/(1 + bodyVariance);
    	Robot.prevX = this.getDistance();
    	/*
    	System.out.println("distance = "+distance+", fallapart= "+fallApart
    			+ ", mainAndEntireDiff= "+mainAndEntireDiff+", mainBodyAngularV"+
    			mainBodyAngularV + ", fit = " + this.fitness + ", this round = "+
    			distance/(1 + fallApart)/(1+mainAndEntireDiff)/(1+mainBodyAngularV));*/
    }
    
    //-----------------------------------------------------------------------------------
    // measurement functions:
    // this will combine all the others
    // Note: the other 4 measurements have to go to fitness - variance, distance, 
    // 		 mainbody variance and mainBody AngularV
    public Boolean isBad(float maxAgeSource, float maxRestAgeSource, Boolean myDemo){
    	Boolean ret = false;
    	Boolean print = false;
    	if(myDemo) print = true;
    	if(isTooOld(maxAgeSource)){
    		if(print) System.out.println("He is dead.");
    		ret = true;
    	}
    	if(isLazy(maxRestAgeSource)){
    		if(print) System.out.println("He is lazy.");
    		ret = true;
    	}
    	return ret;
    }
    public Boolean isTooOld(float maxAgeSource){
    	if (this.age > maxAgeSource){return true;}
    	else return false;
    }
    public Boolean isLazy(float maxRestAgeSource){
    	if (this.restAge > maxRestAgeSource){return true;}
    	else return false;
    }
    //-----------------------------------------------------------------------------------
    
    public MyResult getResult(World w){	
    	MyResult res;
    	//if (this.fitness < 0f) this.fitness = 0f;
        
    	// 1. energy use
    	float totalEnergyUseRatio = 0f;
        if(totalEnergyUseBase != 0f) totalEnergyUseRatio = totalEnergyUse / totalEnergyUseBase;
        else assert(totalEnergyUse == 0f);
    	/*System.out.println("totalEnergyUseBase = "+totalEnergyUseBase);
    	System.out.println("totalEnergyUse = "+totalEnergyUse);
    	System.out.println("totalNumberOfContact = "+totalNumberOfContact);
    	System.out.println("contactNumberBase = "+contactNumberBase);
    	System.out.println("totalEnergyUseRatio = "+totalEnergyUseRatio);*/
    	assert(totalEnergyUseRatio <= 1f && totalEnergyUseRatio >= 0f);
    	fitness /= (1+totalEnergyUseRatio);
    	
    	// 2. more joint is better, it is more flexible
    	float jointRatio = jointPool.size()/MyConfiguration.maxJoints;
    	assert(jointRatio>=0 && jointRatio<=1);
    	fitness *= (2+jointRatio);
    	
    	// 3. velocity variance
    	float aveVelocity = totalVelocity/numberOfVelocity;
    	float variance = 0f;
    	for(float v : velocities){
    		variance += (v-aveVelocity)*(v-aveVelocity);
    	}
    	variance /= numberOfVelocity;
    	float stdDev = (float) Math.sqrt(variance);
    	//System.out.println(stdDev);
    	// normally stdDev is 1~2, for demo is bigger
    	fitness/=(1+stdDev);
    	
    	// 4. the total area of the robot - we should keep the materials low
    	//    we can use mass to simulate the area since all the densities are same
    	float totalMass = 0f;
    	for(Body b : bodyPool){
    		totalMass += b.m_mass;
    	}
    	//System.out.println("mass = "+totalMass);
    	assert(totalMass <= 100f);
    	float massRatio = totalMass / 100f;
    	fitness/=(1+massRatio);
    	
    	// final result
    	res = new MyResult(myPartPool, myJointPool, this.getDistance(), this.getAge(), 
    			this.totalEnergyUse, this.fitness);
        return res;
    }
    
    public void destroy(World w){
        for (Joint joint : jointPool) {w.destroyJoint(joint);}
        for (Body body : bodyPool) {w.destroyBody(body);}
    }
    
    //-----------------------------------------------------------------------------------
    public ArrayList<Body> getBodyPool(){return bodyPool;}
    public ArrayList<MyJoint> getMyJointPool(){return myJointPool;}
    public ArrayList<RevoluteJoint> getJointPool(){return jointPool;}
    public Body getMainBody(){return mainBody;}
    public float getAge(){ return this.age;}
    public void addRestAge(float time){this.restAge += time;}
    public float getRestAge(){ return this.restAge;}
    // walking distance
    public float getDistance(){
    	// we should calculate the average position of all the Bodies of this robot
    	Vec2 position = new Vec2(0f, 0f);
    	int counter = 0;
    	float minX = 1000f;
		for (Body bodyTemp : bodyPool) {
			if (bodyTemp.getType() == BodyType.DYNAMIC){
				counter++;
				float x = bodyTemp.getPosition().x;
				float y = bodyTemp.getPosition().y;
				if(Float.isNaN(x) || Float.isInfinite(x) || Float.isNaN(y) || Float.isInfinite(y)){
					return 0f;
				}
			    position.x += x;
			    position.y += y;
				if(x < minX) minX = x;
			}
		}
		position.x /= counter;
		position.y /= counter;
		//if(position.x-Robot.bodyX < 0) return 0;
		//System.out.println("position.x = "+position.x+", Robot.bodyX = "+Robot.bodyX);
		//return position.x-Robot.bodyX;
		//System.out.println("position.x = "+position.x+", Robot.bodyX = "+Robot.bodyX + "minx = "+minX);
		return minX-Robot.bodyX;
    }
}
