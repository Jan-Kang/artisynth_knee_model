package lowerlimb;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.NodeNumberReader;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.SlottedHingeJoint;
import artisynth.core.renderables.ColorBar;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

public class lowerlimb extends RootModel {
	
	//Path of data
	String geodata = PathFinder.getSourceRelativePath(this, "data/");
	
	//create mech
	MechModel mech = new MechModel();

	//create fem
    FemModel3d femur;
    FemModel3d tifi;
    FemModel3d cartilage;
    
    Color myJointColor = new Color (93/255f, 93/255f, 168/255f);
    
	@Override
	public void build(String[] args) throws IOException {
		
        //close of Gravity
        mech.setGravity(0, 0, 0);
		
		//import femur
	    femur = new FemModel3d("femur");
		femur = AnsysCdbReader.read(geodata+"Femur.cdb");
		
        //Set physical properties
        femur.setDensity(1900);
        femur.setMassDamping(0.01);
        femur.setStiffnessDamping(0.02);
        femur.setMaterial(new LinearMaterial(1e9, 0.3));
        
        //calculates the femur size
        computeBoundingBox(femur);
		
		//import tibi
	    tifi = new FemModel3d("tifi");
		tifi = AnsysCdbReader.read(geodata+"TibiaFibula.cdb");
		
        //Set physical properties
        tifi.setDensity(1900);
        tifi.setMassDamping(0.01);
        tifi.setStiffnessDamping(0.02);
        tifi.setMaterial(new LinearMaterial(1e9, 0.3));
        
        //calculates the tibi size
        computeBoundingBox(tifi);
        
        //create cartilage
//        cartilage = new FemModel3d("cartilage");
//
//        double radius = 100;  
//        double thickness = 1; 
//        
//        FemFactory.createCylinder(cartilage, thickness, radius, 20, 10, 20); // 创建一个圆柱，20为圆的细分数
//
//        cartilage.setDensity(1000);  
//        cartilage.setMaterial(new LinearMaterial(1e7, 0.4));  
//
//        Point3d cartilagePosition = new Point3d(
//            (36.6293 + 122.6768) / 2,  
//            (-68.1622 + 51.3401702) / 2,
//            -435.0917            
//        );
//        
//        for (FemNode3d node : cartilage.getNodes()) {
//            Point3d currentPosition = node.getPosition();
//            node.setPosition(new Point3d(
//                currentPosition.x + cartilagePosition.x,
//                currentPosition.y + cartilagePosition.y,
//                currentPosition.z + cartilagePosition.z
//            ));
//        }
//        
//        mech.addModel(cartilage);
//
//        RenderProps.setFaceColor(cartilage, new Color(0.5f, 0.8f, 1f));  
//        RenderProps.setLineColor(cartilage, Color.BLUE); 
        
        //add models
        mech.addModel(femur);
        mech.addModel(tifi);
        //mech.addModel(cartilage);
        
        //open collisionBehavior
        //mech.setCollisionBehavior(femur, tifi, true);
        
        addModel(mech);
        
        //set render properties
        setRenderProps (femur);
        setRenderProps (tifi);
        
        // create a slotted revolute joint that connects the two fem
        RigidTransform3d TDW = new RigidTransform3d(    
        					(36.6293 + 122.6768) / 2,  
        					(-68.1622 + 51.3401702) / 2,
        					-435.0917 ,
        					-Math.PI/3, 0, Math.PI/2);
        SlottedHingeJoint joint = new SlottedHingeJoint (femur, tifi, TDW);
        mech.addBodyConnector (joint);
        
        // set ranges and rendering properties for the joint
        joint.setShaftLength (70);
        joint.setMinX (-0.1);
        joint.setMaxX (0.1);
        joint.setSlotDepth (0.63);
        joint.setSlotWidth (0.08);
        RenderProps.setFaceColor (joint, myJointColor);
	}
	
	protected void setRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering(SurfaceRender.Shaded);
		RenderProps.setLineColor (fem, Color.red);
		RenderProps.setFaceColor(fem, new Color (0.5f, 0.5f, 1f));
	}
	
	//calculates the modul size
	public void computeBoundingBox(FemModel3d fem) {

	    Point3d min = new Point3d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
	    Point3d max = new Point3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

	    for (FemNode3d node : fem.getNodes()) {
	        Point3d pos = node.getPosition();

	        if (pos.x < min.x) {
	            min.x = pos.x;
	        }
	        if (pos.y < min.y) {
	            min.y = pos.y;
	        }
	        if (pos.z < min.z) {
	            min.z = pos.z;
	        }

	        if (pos.x > max.x) {
	            max.x = pos.x;
	        }
	        if (pos.y > max.y) {
	            max.y = pos.y;
	        }
	        if (pos.z > max.z) {
	            max.z = pos.z;
	        }
	    }

	    System.out.println("Model Bounding Box:");
	    System.out.println("Min: " + min.toString());
	    System.out.println("Max: " + max.toString());
	    
	    double width = max.x - min.x;
	    double height = max.y - min.y;
	    double depth = max.z - min.z;
	    
	    System.out.println("Model Dimensions (Width, Height, Depth): " + width + ", " + height + ", " + depth);
	}
	
}
