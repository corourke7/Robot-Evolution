package main;
/* Basic program structure is based on corazza/Robot-Evolution Github 
* https://github.com/corazza/Robot-Evolution
* @author Shen Li, Colleen O'Rourke
*/
import org.newdawn.slick.AppGameContainer;
import org.newdawn.slick.SlickException;
import org.newdawn.slick.opengl.renderer.Renderer;

import gui.MyDemo;
 

public class Main {

	public static void main(String[] args) throws Exception {
		// start!
    	try{
    		Renderer.setRenderer(Renderer.VERTEX_ARRAY_RENDERER);
			//AppGameContainer container = new AppGameContainer(new MyDemo());
			AppGameContainer container = new AppGameContainer(new MyDemo());
			container.setVSync(true);
			container.setTargetFrameRate(60);
			container.setDisplayMode(MyDemo.windowDimensions[0],MyDemo.windowDimensions[1],false);
			container.start();
		}
    	catch (SlickException e) {e.printStackTrace();}
	}

}














