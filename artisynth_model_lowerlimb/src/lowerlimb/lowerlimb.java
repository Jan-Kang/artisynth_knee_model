package lowerlimb;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.NodeNumberReader;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.ConnectableBody;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.HingeJoint;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.SlottedHingeJoint;
import artisynth.core.renderables.ColorBar;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

public class lowerlimb extends RootModel {
	// Path of data
	String ModelData = PathFinder.getSourceRelativePath(this, "data/");	
	// create Mech
	MechModel mech = new MechModel ();
	// create Fem
    FemModel3d Femur;
    FemModel3d TiFi;
    // create a Joint between Femur and TibiaFibula 
    JointBase JointFTF;
 
	@Override
	public void build(String[] args) throws IOException {
        // Gravity
        mech.setGravity (0, 0, -9.81);
        // Model
        Femur = importFemur ();
        TiFi = importTibiaFibula ();
        addModel (mech);
        // Joint
        JointFTF = createJoint (Femur, TiFi);
        JointFTF.setEnabled(false);
        // Joint display
        JointDisplay(JointFTF);
        // Contact
        mech.setCollisionBehavior(Femur, TiFi, true, 0);
        
	}
	
	// Import FEM model 
	private FemModel3d importFemur () throws IOException {
		// import Femur
		FemModel3d Femur = null;
		Femur = new FemModel3d ("Femur");
		Femur = AnsysCdbReader.read (ModelData + "Femur.cdb");
		// set physical properties
		Femur.setDensity (1.9e-6);
		Femur.setMassDamping (0.01);
		Femur.setStiffnessDamping (0.02);
		Femur.setMaterial(new LinearMaterial (1e9, 0.3));
		Femur.setName ("Femur");
		if (Femur.isVolumeValid ())
			System.out.println ("Femur mesh valid.");
		// calculates the Femur size
		ModelSize (Femur);
		mech.addModel (Femur);
		setFemRenderProps (Femur);
		return Femur;
	}
	private FemModel3d importTibiaFibula () throws IOException {
		// import TibiaFibula
		FemModel3d TibiaFibula = null;
		TibiaFibula = new FemModel3d ("TibiaFibula");
		TibiaFibula = AnsysCdbReader.read (ModelData + "TibiaFibula.cdb");
		// set physical properties
		TibiaFibula.setDensity (1.9e-6);
		TibiaFibula.setMassDamping (0.01);
		TibiaFibula.setStiffnessDamping (0.02);
		TibiaFibula.setMaterial (new LinearMaterial (1e9, 0.3));
		TibiaFibula.setName ("TibiaFibula");
		if (TibiaFibula.isVolumeValid())
			System.out.println ("TibiaFibula mesh valid.");
		// Fix all nodes of the model
		for (FemNode3d node : TibiaFibula.getNodes()) {
			node.setDynamic(false);
		}
		// calculates the TibiaFibula size
		ModelSize (TibiaFibula);
		mech.addModel (TibiaFibula);
		setFemRenderProps(TibiaFibula);
		return TibiaFibula;
	}
	// Render FemModel
	private void setFemRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering(SurfaceRender.Shaded);
		RenderProps.setLineColor (fem, Color.red);
		RenderProps.setFaceColor(fem, new Color (0.5f, 0.5f, 1f));
	}
	
	// Joint
	private JointBase createJoint (ConnectableBody femur, ConnectableBody tifi) {
		// create a HingeJoint joint that connects the two fem
		Vector3d origin = new Vector3d (82.891323, -35.194814, -421.43464);
		RigidTransform3d TDW =
				new RigidTransform3d (origin.x, origin.y, origin.z);
		TDW.setRpyDeg (0, 90, -20);
		HingeJoint joint = new HingeJoint (tifi, femur, TDW);
		mech.addBodyConnector (joint);
		if (femur instanceof FemModel3d) {
			setJointComplianceForFEM(joint); 
		}
		else {
			setJointComplianceForFrames(joint);
		}
		setJointRenderProps (joint);
		return joint;
	}
 	private void setJointComplianceForFrames (JointBase joint) {
		Frame bodyA = (Frame)joint.getBodyA ();
		Frame bodyB = (Frame)joint.getBodyA ();
		VectorNd comp = new VectorNd (joint.numConstraints ());
		VectorNd damp = new VectorNd (joint.numConstraints ());
		double mass = bodyA.getEffectiveMass () + bodyB.getEffectiveMass ();
		for (int i = 0; i < joint.numConstraints (); i++) {
			comp.set (i, 1);
			damp.set (i, 2 * 1 * Math.sqrt (mass / comp.get (i)));
		}
		joint.setCompliance (comp);
		joint.setDamping (damp);
	}
	private void setJointComplianceForFEM (JointBase joint) {
		FemModel3d meshA = (FemModel3d)joint.getBodyA ();
		FemModel3d meshB = (FemModel3d)joint.getBodyB ();
		VectorNd comp = new VectorNd (joint.numConstraints ());
		VectorNd damp = new VectorNd (joint.numConstraints ());
		double mass = meshA.getActiveMass () + meshB.getActiveMass ();
		for (int i = 0; i < joint.numConstraints (); i++) {
			comp.set (i, 100);
			damp.set (i, 2 * 1 * Math.sqrt (mass / comp.get (i)));
		}
		joint.setCompliance (comp);
		joint.setDamping (damp);
	}
	// Render Joint
	private void setJointRenderProps (JointBase joint) {
		joint.setShaftLength (70);
		joint.setShaftRadius (1);
		joint.setDrawFrameC (AxisDrawStyle.ARROW);
		joint.setDrawFrameD (AxisDrawStyle.ARROW);
		joint.setAxisLength (50);
	}
	// Display Joint
	private void JointDisplay (JointBase joint) {
	    ControlPanel panel = new ControlPanel ("Joint angles");
	    panel.addWidget ("Hinge joint angle", joint, "theta");
	    mech.add (panel);
	}
	
	// Contact
	
	
	// calculates the model size
	private void ModelSize (FemModel3d fem) {

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

	    System.out.println("Model Size:");
	    System.out.println("Min: " + min.toString());
	    System.out.println("Max: " + max.toString());
	    
	    double width = max.x - min.x;
	    double height = max.y - min.y;
	    double depth = max.z - min.z;
	    
	    System.out.println("Model Dimensions (Width, Height, Depth): " + width + ", " + height + ", " + depth);
	}
	
}
