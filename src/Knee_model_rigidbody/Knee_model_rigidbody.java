package Knee_model_rigidbody;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.ConnectableBody;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.GimbalJoint;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.renderables.ColorBar;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;

public class Knee_model_rigidbody extends RootModel {
	// path of data
	String Modeldata = maspack.util.PathFinder.getSourceRelativePath(this, "data/");
	// create MECH
	MechModel mech = new MechModel ();
	// create Rigid
	RigidBody Femur, FemurCart, 
	  		  Mensicus, 
	  		  TibiaCart, TibiaFibula,
	  		  Patella, PatellaCart;
    // create joint
    JointBase Joint;
	// create collision behaviors
	CollisionBehavior behav;
	public void build (String [] args) throws IOException {
		// set a gravity
		mech.setGravity (0, -9.81, 0);
		// import rigid model
		Femur = importFemur();
		FemurCart = importFemurCart();
		TibiaFibula = importTibiaFibula();
		TibiaCart = importTibiaCart();
		Mensicus = importMensicus();
		Patella = importPatella();
		PatellaCart = importPatellaCart();
		addModel(mech);
		// set a joint
        Joint = createJoint (Femur, TibiaFibula);
		// set Collision
		behav = new CollisionBehavior();
		mech.setCollisionBehavior(TibiaFibula, Femur, true, 0);
		behav.setCompliance(0.1);
		behav.setDamping(100);
		// create a color bar
		ColorBar cbar = new ColorBar ();
		cbar.setName ("colorBar");
	    cbar.setNumberFormat ("%.2f");
	    cbar.populateLabels (0.0, 0.1, 10);
	    cbar.setLocation (-100, 0.1, 20, 0.8);
	    addRenderable (cbar);
	}

	// import Rigid model
	private RigidBody importFemur () throws IOException {
		PolygonalMesh meshFemur = null;
		meshFemur = new PolygonalMesh (Modeldata + "femur.obj");
		RigidBody femur = 
				RigidBody.createFromMesh("femur", meshFemur, 1800, 1);
		mech.addRigidBody(femur);
		return femur;
	}
	private RigidBody importTibiaFibula () throws IOException {
		PolygonalMesh meshTibiaFibula = null;
		meshTibiaFibula = new PolygonalMesh (Modeldata + "tibia.obj");
		RigidBody TibiaFibula = 
				RigidBody.createFromMesh("TibiaFibula", meshTibiaFibula, 1800, 1);
		mech.addRigidBody(TibiaFibula);
		TibiaFibula.setDynamic(false);
		return TibiaFibula;
	}
	private RigidBody importFemurCart () throws IOException {
		PolygonalMesh meshFemurCart = null;
		meshFemurCart = new PolygonalMesh (Modeldata + "femcart.obj");
		RigidBody FemurCart = 
				RigidBody.createFromMesh("FemurCart", meshFemurCart, 1100, 1);
		mech.addRigidBody(FemurCart);
		return FemurCart;
	}
	private RigidBody importTibiaCart () throws IOException {
		PolygonalMesh meshTibiaCart = null;
		meshTibiaCart = new PolygonalMesh (Modeldata + "tibcart.obj");
		RigidBody TibiaCart = 
				RigidBody.createFromMesh("TibiaCart", meshTibiaCart, 1100, 1);
		mech.addRigidBody(TibiaCart);
		return TibiaCart;
	}
	private RigidBody importMensicus () throws IOException {
		PolygonalMesh meshMensicus = null;
		meshMensicus = new PolygonalMesh (Modeldata + "mensicus.obj");
		RigidBody Mensicus = 
				RigidBody.createFromMesh("Mensicus", meshMensicus, 1100, 1);
		mech.addRigidBody(Mensicus);
		return Mensicus;
	}
	private RigidBody importPatella () throws IOException {
		PolygonalMesh meshPatella = null;
		meshPatella = new PolygonalMesh (Modeldata + "patella.obj");
		RigidBody Patella = 
				RigidBody.createFromMesh("Patella", meshPatella, 1100, 1);
		mech.addRigidBody(Patella);
		return Patella;
	}
	private RigidBody importPatellaCart () throws IOException {
		PolygonalMesh meshPatellaCart = null;
		meshPatellaCart = new PolygonalMesh (Modeldata + "patcart.obj");
		RigidBody PatellaCart = 
				RigidBody.createFromMesh("PatellaCart", meshPatellaCart, 1100, 1);
		mech.addRigidBody(PatellaCart);
		return PatellaCart;
	}
	// set a Render for rigid body

	// create joint
	private JointBase createJoint (ConnectableBody femur, ConnectableBody tifi) {
		Vector3d origin = new Vector3d (360, 1380, 840);
		RigidTransform3d TDW =
				new RigidTransform3d (origin.x, origin.y, origin.z);
		TDW.setRpyDeg (0, 90, 0);
		GimbalJoint joint = new GimbalJoint (femur, tifi, TDW);
	    // set joint ranges (in degrees)
		joint.setRollRange (-90, 0);
	    joint.setPitchRange (-5, 5);
	    joint.setYawRange (-5, 5);
	    // set joint initial value
	    // joint.setRoll (0);
	    // joint.setPitch (0);
	    // joint.setYaw (0);
		// set joint lock
		joint.setPitchLocked(true);
		joint.setYawLocked(true);
		mech.addBodyConnector (joint);
		if (femur instanceof FemModel3d) {
			setJointComplianceForFEM(joint); 
		}
		else {
			setJointComplianceForFrames(joint);
		}
		setJointRenderProps (joint);
		JointControl(joint);
		return joint;
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
 	private void setJointComplianceForFrames (JointBase joint) {
		Frame bodyA = (Frame)joint.getBodyA ();
		Frame bodyB = (Frame)joint.getBodyB ();
		VectorNd comp = new VectorNd (joint.numConstraints ());
		VectorNd damp = new VectorNd (joint.numConstraints ());
		double mass = bodyA.getEffectiveMass () + bodyB.getEffectiveMass ();
		for (int i = 0; i < joint.numConstraints (); i++) {
			comp.set (i, 100);
			damp.set (i, 2 * 1 * Math.sqrt (mass / comp.get (i)));
		}
		joint.setCompliance (comp);
		joint.setDamping (damp);
	}
 	// set joint RenderProps
	private void setJointRenderProps (JointBase joint) {
		joint.setShaftLength (70);
		joint.setShaftRadius (1);
		joint.setAxisLength (50);
		joint.setDrawFrameC (AxisDrawStyle.ARROW);
		joint.setDrawFrameD (AxisDrawStyle.ARROW);
	}
	// set control Joint
	private void JointControl (JointBase joint) {
	    ControlPanel panel = new ControlPanel();
	    panel.addWidget (joint, "roll");
	    panel.addWidget (joint, "pitch");
	    panel.addWidget (joint, "yaw");
	    panel.addWidget (joint, "rollRange");
	    panel.addWidget (joint, "pitchRange");
	    panel.addWidget (joint, "yawRange");
	    panel.addWidget (joint, "rollLocked");
	    panel.addWidget (joint, "pitchLocked");
	    panel.addWidget (joint, "yawLocked");
	    panel.addWidget (joint, "drawFrameC");
	    panel.addWidget (joint, "drawFrameD");
	    panel.addWidget (joint, "axisLength");
	    panel.addWidget (joint, "jointRadius");
	    panel.addWidget (joint, "linearCompliance");
	    panel.addWidget (joint, "rotaryCompliance");
	    panel.addWidget (joint, "compliance");
	    panel.addWidget (joint, "damping");
	    addControlPanel (panel);
	}
}
