package gui;

import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jgap.Chromosome;
import org.jgap.Configuration;
import org.jgap.FitnessFunction;
import org.jgap.Gene;
import org.jgap.Genotype;
import org.jgap.IChromosome;
import org.jgap.InvalidConfigurationException;
import org.jgap.UnsupportedRepresentationException;
import org.jgap.data.DataTreeBuilder;
import org.jgap.data.IDataCreators;
import org.jgap.impl.DefaultConfiguration;
import org.jgap.xml.GeneCreationException;
import org.jgap.xml.ImproperXMLException;
import org.jgap.xml.XMLDocumentBuilder;
import org.jgap.xml.XMLManager;
import org.newdawn.slick.BasicGame;
import org.newdawn.slick.Color;
import org.newdawn.slick.GameContainer;
import org.newdawn.slick.Graphics;
import org.newdawn.slick.Image;
import org.newdawn.slick.SlickException;
import org.newdawn.slick.geom.Polygon;
import org.w3c.dom.Document;
import org.xml.sax.SAXException;

import evolution.MyChromosome;
import evolution.MyFitnessFunct;
import evolution.MyJoint;
import evolution.MyPart;
import simulation.MyResult;
import simulation.Robot;
import simulation.Simulation;
import util.MyConfiguration;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;


/**
 * JBox2D use meter and openGL uses pixel
 * 30 pixels = 1 metre
 *
 * @author Oskar Veerhoek
 */
public class MyDemo extends BasicGame{

	private static final String windowTitle = MyConfiguration.windowTitle;
    // this is in pixel
    public static final int[] windowDemensions = MyConfiguration.windowDemensions;
    // this is in pixel
    // 30 pixels = 1 metre
    private static final float pixelsPerMeter = MyConfiguration.pixelsPerMeter;
    // text position in pixel
    // for drawstring function, the origin is at left top corner.
    private static final int[] textPosition = MyConfiguration.textPosition;
    


	// part pool
	private static ArrayList<MyPart> myPartPool;
	// Body pool in JBox2D
	private static ArrayList<Body> bodyPool;
	// color
	private static ArrayList<Integer> color;
	// joint pool
	private static ArrayList<MyJoint> myJointPool;
	// Joint pool in JBox2D
	private static ArrayList<RevoluteJoint> jointPool;
	// which part is the body, we have to keep it steady
	private static int mainBodyPartIndex;
	
	// the world
    private static World world;
    // in demo, we don't make it too fast
	public static final float timeStep = MyConfiguration.myDemoTimeStep;
    // the ground
    private static Body ground;
    // ground length and height in meters
    public static final float[] groundSizeMeter = MyConfiguration.groundSizeMeter;
    // the robot
    private static Robot robot;
    private static Body mainBody;
    // we need to kill this robot in finite iterations in demo
 	// Note: this is different from the maxIterationCount in Simulation, because we want to see longer demos
 	private static final int maxDemoIterationCount = MyConfiguration.maxDemoIterationCount;
 	// Note: these 2 are different from the maxIterationCount in Simulation, because we want to see longer demos
 	public static final float maxAge = MyConfiguration.maxDemoAge;
    public static final float maxRestAge = MyConfiguration.maxDemoRestAge;
    
 	// we should kill this demo sometimes
 	@SuppressWarnings("unused")
	private static Boolean killDemo = false;

 	// save and load
	private static String populationSavePath;
	private static String chromosomeSavePath;
	private static String populationLoadPath;
	private static String chromosomeLoadPath;
	private static final int populationSize = MyConfiguration.populationSize;
	private static final int maxEvolutionsPerLoop = MyConfiguration.maxEvolutionsPerLoop;

	private static IChromosome bestChromosome;
	private static Genotype bestPopulation;
	private Gene[] genesArray;
	
	private Configuration conf;
	private FitnessFunction myFunc;
	
	// the list of all the outputs we are showing on the screen currently
    private static LinkedList<String> outputList;
    // every screen we show 20 outputs
    private static final int outputListLength = MyConfiguration.outputListLength;
    // how thick is one line will take
    private static final int lineHeight = MyConfiguration.lineHeight;
    
    // count how many generations have been done
    private static int generationCount;

    // the status in Update 
    private static int status;
    private static final int initRobotStatus = 0;
    private static final int runRobotStatus = 1;
    private static final int evolveStatus = 2;
    private static int robotIterationCount;
    private static int evolutionIterationCount;
    
    private static Image background;
    
    // the 0,0 coordinate is in the top left corner.
    public MyDemo() {super(windowTitle);}
   
	@Override
	public void render(GameContainer container, Graphics g) throws SlickException{
		if(status == runRobotStatus){robotRenderer(g);}
		else if(status == evolveStatus){outputRenderer(g);}
		else{}
    }

	@Override
	public void init(GameContainer arg0) throws SlickException {
		 /* the JBox2D world is different from the projection world
         * JBox2D uses meter and projection world uses pixel
         * if we use: 30 pixel = 1 meter, projection matrix = 0, 512, 0, 256 ... (glOrtho )
         * 		projection world origin = (512/2, 256/2)
         * 		JBox2D world origin = (512/2/30, 256/2/30)
         */
    	// 1. set up some numbers
		robotIterationCount = 0;
		evolutionIterationCount = 0;
		status = initRobotStatus;
        outputList = new LinkedList<String>();
        generationCount = 1;
        
        myPartPool = new ArrayList<MyPart>();
        bodyPool = new ArrayList<Body>();
        color = new ArrayList<Integer>();
        myJointPool = new ArrayList<MyJoint>();
        jointPool = new ArrayList<RevoluteJoint>();
        
    	try {background = new Image("background.png");}
    	catch (SlickException e) {e.printStackTrace();}
        // 2. load the chromosomes and update related info
     	try{setupEvolution();} 
     	catch (InvalidConfigurationException e) {e.printStackTrace();}
     	try{load();}
     	catch (InvalidConfigurationException | IOException | SAXException
				| ImproperXMLException | GeneCreationException
				| UnsupportedRepresentationException e) {e.printStackTrace();}
     	retrieveGoodInfoFromChromosome(bestChromosome);
     	setUpWorld();
	}

	@Override
	public void update(GameContainer arg0, int arg1) throws SlickException {

		if(status == initRobotStatus){
			MyFitnessFunct mm = new MyFitnessFunct();
			System.out.println("@@@Generation = " + new Integer(generationCount).toString() 
					+ ", Fitness = " + mm.evaluate(bestChromosome));
			status = runRobotStatus;
		}
		else if(status == runRobotStatus){
			robotIterationCount++;
			if (robotIterationCount > MyDemo.maxDemoIterationCount /*|| killDemo == true*/) {
	    		robotIterationCount = 0;
	            killDemo = false;
	            robot.getResult(MyDemo.world);
	            robot.destroy(MyDemo.world);
				status = evolveStatus;
			}
			else{
		        world.step(MyDemo.timeStep, Simulation.velocityIterations, Simulation.positionIterations);
		    	Simulation.motorMoving(robot);
		    	if(Simulation.robotDead(robot, true) == true){
		    		System.out.println("Demo is dead");
		    		killDemo = true;
		    	}
//	    		System.out.println("joint = "+robot.getJointPool().size());
//	    		System.out.println("body = "+robot.getBodyPool().size());
			}
		}
        // 2. evolve and draw output from evolution
		else if(status == evolveStatus){
			
			evolutionIterationCount++;
            if(evolutionIterationCount < maxEvolutionsPerLoop){	
            	generationCount++;
    			bestPopulation.evolve();
    			bestChromosome = bestPopulation.getFittestChromosome();
    			// we keep the list length
    			MyFitnessFunct m = new MyFitnessFunct();
    			m.evaluate(bestChromosome);
    			MyResult res = m.getResult();
    			if (outputList.size() >= outputListLength) outputList.removeFirst();
    			outputList.add(new String("Generation = " + new Integer(generationCount).toString() 
    					+ ", Fitness = " + res.getFitness() + ", Age = " + res.getAge()
    					+ ", Distance = " + res.getDistance()));
    			//System.out.println( "The best solution contained the following: " );
    			//System.out.println(bestChromosome);
    		}
            else{
                try {save();} catch (Exception e) {e.printStackTrace();}
                retrieveGoodInfoFromChromosome(bestChromosome);
                setUpWorld();
                status = initRobotStatus;
                evolutionIterationCount = 0;
            }
    	}
	}

	//----------------------------------------------------------------------------
	// renderers
    private static void robotRenderer(Graphics g){
    	/*
    	 * The slick2D coordinate:
    	 *   _______________________\  X 
    	 *   |	(0,0)				/  1024 pixels
    	 *   |
    	 * 	 |
    	 * 	 |
    	 *   |
    	 *   |
    	 *  \|/ Y 512 pixels
    	 *   
    	 * 
    	 * The JBox2D coordinate:
    	 * Y/|\512/30 meters
    	 * 	 |
    	 * 	 |
    	 *   |
    	 *   |
    	 *   |
    	 *   |
    	 *   |______________________\  X
    	 * (0,0) 					/  1024/30 meters
    	 */
    	g.drawImage(background, 0, 0);
    	// 1. draw info
    	g.setColor(Color.yellow);
		g.drawString("X = "+String.format("%.2f", robot.getDistance()) + " meters", 
    			textPosition[0], textPosition[1]);
    	g.drawString("Age = " + String.format("%.2f", robot.getAge()) + "seconds", 
    			textPosition[0], textPosition[1]+20);
    	g.drawString(robot.getJointPool().size() + " joints and " + robot.getBodyPool().size()
    			+ " parts in total", textPosition[0], textPosition[1]+40);
    	
    	// 2. draw ground
        Vec2 groundPosition = new Vec2(convertMeterToPixel(ground.getPosition().x), 
        		convertMeterToPixel(ground.getPosition().y));
        Polygon groundShape = new Polygon();
        groundShape.addPoint(groundPosition.x - convertMeterToPixel(groundSizeMeter[0])/2, 
        		windowDemensions[1]-(groundPosition.y - convertMeterToPixel(groundSizeMeter[1])/2));
        groundShape.addPoint(groundPosition.x + convertMeterToPixel(groundSizeMeter[0])/2, 
        		windowDemensions[1]-(groundPosition.y - convertMeterToPixel(groundSizeMeter[1])/2));
        groundShape.addPoint(groundPosition.x + convertMeterToPixel(groundSizeMeter[0])/2, 
        		windowDemensions[1]-(groundPosition.y + convertMeterToPixel(groundSizeMeter[1])/2));
        groundShape.addPoint(groundPosition.x - convertMeterToPixel(groundSizeMeter[0])/2, 
        		windowDemensions[1]-(groundPosition.y + convertMeterToPixel(groundSizeMeter[1])/2));
    	g.setColor(new Color(139,69,19));
    	g.fill(groundShape);
		
        
        // 3. draw Bodies in different color, Bodies connected by 1 joint = 1 color
        color = new ArrayList<Integer>();
    	for (int x = 0; x<bodyPool.size();x++){
        	color.add(0);
    	}
        int colorIndex = 0;
        for (Joint j : jointPool){
        	Body A = j.getBodyA();
        	Body B = j.getBodyB();
        	for (int x = 0; x<bodyPool.size();x++){
        		Body b = bodyPool.get(x);
        		if(b.equals(A) || b.equals(B)){
        			color.set(x, colorIndex);
        		}
        	}
        	colorIndex++;
        }
    	for (int x = 0; x<bodyPool.size();x++){
    		Body body = bodyPool.get(x);
    		if (body.getType() == BodyType.DYNAMIC){
                // Color --------------------------------
    			assert(mainBody != null);
                if(body.equals(mainBody))g.setColor(Color.red);
                switch(color.get(x)){
                	case 0:{g.setColor(Color.blue); break;}
                	case 1:{g.setColor(Color.white); break;}
                	case 2:{g.setColor(Color.gray); break;}
                	case 3:{g.setColor(Color.yellow); break;}
                	case 4:{g.setColor(Color.cyan); break;}
                	case 5:{g.setColor(Color.green); break;}
                	case 6:{g.setColor(Color.orange); break;}
                	case 7:{g.setColor(Color.pink); break;}
                	case 8:{g.setColor(Color.lightGray); break;}
                	case 9:{g.setColor(Color.magenta); break;}
                	default:{g.setColor(Color.darkGray); break;}
                }
            	// get vertices in world coordinate instead of local coordinate
            	// http://box2d.org/forum/viewtopic.php?f=3&t=1933
            	PolygonShape shape = (PolygonShape)body.getFixtureList().getShape();
            	Vec2[] vertices = shape.getVertices();
            	ArrayList<Vec2> worldPoints = new ArrayList<Vec2>();
            	int numberOfVertices = shape.getVertexCount();
            	for (Vec2 vec : vertices){
            		worldPoints.add(body.getWorldPoint(vec));
            	}
                Polygon bodyShape = new Polygon();
            	for (int i = 0; i < numberOfVertices; i++){
            		bodyShape.addPoint(convertMeterToPixel(worldPoints.get(i).x), 
            				windowDemensions[1]-(convertMeterToPixel(worldPoints.get(i).y)));
            	}	
                g.fill(bodyShape);
                g.setColor(Color.black);
                g.draw(bodyShape);
            }
    	}
    }
	
	// output page
    private static void outputRenderer(Graphics g){
        // then we should draw the 10 strings
        int x0 = textPosition[0];
        int y0 = textPosition[1];
    	// keep track of how many lines we have drawn
        int lineIndex = 0;
		g.setColor(Color.white);
        for(int i = 0; i < outputList.size(); i++){
    		lineIndex++;
    		int y = y0 + lineHeight * (lineIndex-1);
    		g.drawString(outputList.get(i), x0, y);
        }
    }
    
    
	//---------------------------------------------------------------------
	private void setupEvolution() throws InvalidConfigurationException{
		updatePath();
		// present the current chromosome in JBox2D
		// start the genetic algorithm and present it on screen
		//MyEvolution me = new MyEvolution();
		//MyEvolutionRetVal retVal = me.evolve();
		// http://jgap.sourceforge.net/doc/tutorial.html#step3
		// Start with a DefaultConfiguration, which comes setup with the most common settings.
		conf = new DefaultConfiguration();
		// Set the fitness function to 'FitnessFunct'
		myFunc = new MyFitnessFunct ();
		conf.setFitnessFunction(myFunc);
        ArrayList<Gene> genes = new ArrayList<Gene>(MyChromosome.generateGenes(conf));
		genesArray = new Gene[genes.size()];
		genesArray = genes.toArray(genesArray);
	}
    
	private void load() throws InvalidConfigurationException, IOException, SAXException, 
		ImproperXMLException, GeneCreationException, UnsupportedRepresentationException{
		// load population from a file
		try{
			// set up chromosomes and population
	    	Document doc2 = XMLManager.readFile(new File(chromosomeLoadPath));
	        bestChromosome = XMLManager.getChromosomeFromDocument(conf, doc2);
			conf.setSampleChromosome(bestChromosome);
			conf.setPopulationSize(populationSize);
	    	Document doc = XMLManager.readFile(new File(populationLoadPath));
	    	bestPopulation = XMLManager.getGenotypeFromDocument(conf, doc);
		}
		catch (UnsupportedRepresentationException uex){
            // JGAP codebase might have changed between two consecutive runs.
			bestChromosome  = new Chromosome(conf, genesArray);
			conf.setSampleChromosome(bestChromosome);
			conf.setPopulationSize(populationSize);
			bestPopulation = Genotype.randomInitialGenotype(conf);
	    	bestChromosome = bestPopulation.getFittestChromosome();
        }
        catch (FileNotFoundException fex){
			bestChromosome  = new Chromosome(conf, genesArray);
			conf.setSampleChromosome(bestChromosome);
			conf.setPopulationSize(populationSize);
			bestPopulation = Genotype.randomInitialGenotype(conf);
	    	bestChromosome = bestPopulation.getFittestChromosome();
        }
	}
	
	private static void save() throws Exception{
		// http://rongxiwang.blog.163.com/blog/static/588125972010322090404/
        DataTreeBuilder builder = DataTreeBuilder.getInstance();
        IDataCreators doc = builder.representGenotypeAsDocument(bestPopulation);
        IDataCreators doc2 = builder.representChromosomeAsDocument(bestChromosome);
        // create XML document from generated tree
        XMLDocumentBuilder docbuilder = new XMLDocumentBuilder();
        Document xmlDoc = (Document) docbuilder.buildDocument(doc);
        XMLManager.writeFile(xmlDoc, new File(populationSavePath + "Generation" +
        		Integer.toString(generationCount) + ".xml"));
        docbuilder = new XMLDocumentBuilder();
        Document xmlDoc2 = (Document) docbuilder.buildDocument(doc2);
        XMLManager.writeFile(xmlDoc2, new File(chromosomeSavePath + "Chromosome" +
        		Integer.toString(generationCount) + ".xml"));
    }

	// get information back from the chromosomes
	public static void retrieveGoodInfoFromChromosome(IChromosome bestChromosome){
		MyFitnessFunct mff = new MyFitnessFunct();
		mff.retrieveGoodInfo(bestChromosome);
		MyDemo.myPartPool = new ArrayList<MyPart>(mff.getPartPool());
		MyDemo.myJointPool = new ArrayList<MyJoint>(mff.getJointPool());
		MyDemo.mainBodyPartIndex = mff.getMainBodyPartIndex();
	}
	
	private void updatePath(){
	    String basePath = new File("").getAbsolutePath();
	    //System.out.println(basePath);
	    // http://www.coderanch.com/t/531925/java/java/string-literal-properly-closed-double
		populationSavePath = basePath + "\\Populations\\";
		chromosomeSavePath = basePath + "\\Chromosomes\\";
		populationLoadPath = basePath + "\\Populations\\popu.xml";
		chromosomeLoadPath = basePath + "\\Chromosomes\\chromo.xml";
	}
	
    
    private static void setUpWorld() {
    	// world
    	// the gravity on x and y
        Vec2 gravityVec2 = new Vec2(0f, Simulation.gravity);
        world = new World(gravityVec2);
        world.setAllowSleep(true);
        // ground
        // add ground to this world in MyDemo, not that world in Simulation
        Simulation.createGround(world);
        ground = (Body)(world.getBodyList());
        // robot
        robot = new Robot(myPartPool, myJointPool, world, mainBodyPartIndex);
        mainBody = robot.getMainBody(); assert(mainBody!=null);
        jointPool = robot.getJointPool();
        myJointPool = robot.getMyJointPool();

        bodyPool = robot.getBodyPool();
    }
    
    public static float convertPixelToMeter(float pixel){return pixel/pixelsPerMeter;}
    public static float convertMeterToPixel(float meter){return meter*pixelsPerMeter;}

}






