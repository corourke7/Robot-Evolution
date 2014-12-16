package util;
/* Basic program structure is based on corazza/Robot-Evolution Github 
* https://github.com/corazza/Robot-Evolution
* @author Shen Li, Colleen O'Rourke
*/
import java.util.ArrayList;

import org.jbox2d.dynamics.joints.RevoluteJoint;

import evolution.MyJoint;

public class JointSeq {

	private ArrayList<MyJoint> myJointPool;
	private ArrayList<RevoluteJoint> jointPool;
	private ArrayList<Integer> sequence;
	private ArrayList<MyJoint> tempMyJointPool;
	private ArrayList<RevoluteJoint> tempJointPool;
	private ArrayList<Integer> tempSequence;
	
	private int length;
	
	public JointSeq(ArrayList<MyJoint> mjp, ArrayList<RevoluteJoint> jp, ArrayList<Integer> s){
		this.myJointPool = new ArrayList<MyJoint>(mjp);
		this.jointPool = new ArrayList<RevoluteJoint>(jp);
		this.sequence = new ArrayList<Integer>(s);
        this.tempMyJointPool = new ArrayList<MyJoint>(mjp);
        this.tempJointPool = new ArrayList<RevoluteJoint>(jp);
        this.tempSequence = new ArrayList<Integer>(s);
		length = this.sequence.size();
	}
	
	public void sort() {
		assert(myJointPool.size() == jointPool.size());
		assert(myJointPool.size() == sequence.size());
        doMergeSort(0, length - 1);
    }
 
    private void doMergeSort(int lowerIndex, int higherIndex) {
         
        if (lowerIndex < higherIndex) {
            int middle = lowerIndex + (higherIndex - lowerIndex) / 2;
            // Below step sorts the left side of the array
            doMergeSort(lowerIndex, middle);
            // Below step sorts the right side of the array
            doMergeSort(middle + 1, higherIndex);
            // Now merge both sides
            mergeParts(lowerIndex, middle, higherIndex);
        }
    }
 
    private void mergeParts(int lowerIndex, int middle, int higherIndex) {
 
        for (int i = lowerIndex; i <= higherIndex; i++) {
        	tempMyJointPool.set(i, myJointPool.get(i));
        	tempJointPool.set(i, jointPool.get(i));
        	tempSequence.set(i, sequence.get(i));
        }
        int i = lowerIndex;
        int j = middle + 1;
        int k = lowerIndex;
        while (i <= middle && j <= higherIndex){
            if (tempSequence.get(i) <= tempSequence.get(j)) {
            	myJointPool.set(k, tempMyJointPool.get(i));
            	jointPool.set(k, tempJointPool.get(i));
            	sequence.set(k, tempSequence.get(i));
                i++;
            } 
            else{
            	myJointPool.set(k, tempMyJointPool.get(j));
            	jointPool.set(k, tempJointPool.get(j));
            	sequence.set(k, tempSequence.get(j));
                j++;
            }
            k++;
        }
        while (i <= middle) {
        	myJointPool.set(k, tempMyJointPool.get(i));
        	jointPool.set(k, tempJointPool.get(i));
        	sequence.set(k, tempSequence.get(i));
            k++;
            i++;
        }
    }
    
    public ArrayList<MyJoint> getMyJointPool(){return myJointPool;}
    public ArrayList<RevoluteJoint> getJointPool(){return jointPool;}
    public ArrayList<Integer> getSequence(){return sequence;}
}
	

