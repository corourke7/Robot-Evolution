package evolution;
import java.util.ArrayList;

import org.jbox2d.common.Vec2;

import util.MyConfiguration;



// A part of a robot 
// http://www.box2d.org/manual.html
// 	in box2D, the shape should be polygon, circle, chain and edge. 
// 	our part could be Right Triangle or Box so that we can restrict its area
//  There may be multiple joints for each part
public class MyPart {
	// we have to keep the area of part from being too big
    public static final float minWidth = MyConfiguration.minWidth;
    public static final float maxWidth = MyConfiguration.maxWidth;
    public static final float minHeight = MyConfiguration.minHeight;
    public static final float maxHeight = MyConfiguration.maxHeight;
    
    /*
     * triangle or rectangle
     * h |
     * e |
     * i |
     * g |
     * h |
     * t |______________
     * 		width
     * 
     */
 	private int numberOfVertex; 
    
    private float width;
    private float height;
	
    // joints connected to this part
    public ArrayList<MyJoint> partJoints;
    
    // a pointer to the Body object
    //public Body testBody;
    
    // this part should be rotated around a joint, we should set up the joint position
    // the joint position is on a circle according to the angle parameter
    // parameter is [0 to 1)
    // this is all in local coordinate
    // http://mathworld.wolfram.com/TrianglePointPicking.html
    public Vec2 getAnchor(float parameter1, float parameter2) {
    	// we need to make sure that the point is inside the polygon 
    	// so that the polygon will not rotate by itself in demo
    	//return new Vec2(0f,0f);
    	
    	assert(parameter1>=MyConfiguration.minPartParameter&&parameter1<=MyConfiguration.maxPartParameter);
    	assert(parameter2>=MyConfiguration.minPartParameter&&parameter2<=MyConfiguration.maxPartParameter);
    	if(numberOfVertex == 4){
    		Vec2 v1 = new Vec2(width,0f);
    		Vec2 v2 = new Vec2(0f,height);
    		float x = parameter1*v1.x + parameter2*v2.x;
    		float y = parameter1*v1.y + parameter2*v2.y;
	        return new Vec2(x, y);
    	}
    	else{
    		// for RT triangle
//    		Vec2 v1 = new Vec2(width,0f);
//    		Vec2 v2 = new Vec2(0f,height);
//    		float x = parameter1*v1.x + (1-parameter1)*parameter2*v2.x;
//    		float y = parameter1*v1.y + (1-parameter1)*parameter2*v2.y;
//    		return new Vec2(x,y);
    		// for equilateral triangle
    		float theta = (float) (parameter1*Math.PI/3);
    		float rho = parameter2*Math.min(this.height, this.width)/2f*1.732f;
    		float x = (float) (rho * Math.cos(theta));
    		float y = (float) (rho * Math.sin(theta));
    		return new Vec2(x,y);
    	}
    }
    /*
    public Vec2 getAnchor(float percent) {
        float x = 0;
        float y = 0;

        float o = 2 * width + 2 * height;
        float d = percent * o;

        if (d < height) {
            x = 0;
            y = d;
        } else if (d < height + width) {
            x = d - height;
            y = height;
        } else if (d < 2 * height + width) {
            x = width;
            y = height - (d - height - width);
        } else {
            x = width - (d - height * 2 - width);
            y = 0;
        }

        x -= width / 2;
        y -= height / 2;

        // return new Vec2(width, -height);
        return new Vec2(x, y);
    }*/
    
    public float getWidth(){return this.width;}
    public float getHeight(){return this.height;}
    public int getNumberOfVertex(){return this.numberOfVertex;}
    
    public void setWidth(float source){this.width = source;}
    public void setHeight(float source){this.height = source;}
    public void setNumberOfVertex(int source){
    	if (source != 3 && source != 4){
    		throw new IllegalArgumentException("ERROR: setNumberOfVertex - parameter out of range.");
    	}
    	else this.numberOfVertex = source;
    }
    
    @Override
    public String toString() {
        return "Part [width=" + this.width + ", height=" + this.height 
        		+ ", numberOfVertex=" + this.numberOfVertex + "]";
    }
    
}












