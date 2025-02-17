package Knee_model_rigidbody;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.ConnectableBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.GimbalJoint;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.util.PathFinder;

public class Knee_model_rigidbody extends RootModel {
	
	String Modeldata = maspack.util.PathFinder.getSourceRelativePath(this, "data/");
	MechModel mech = new MechModel ();
	RigidBody Femur, FemurCart, Meniscus, TibiaCart, TibiaFibula, Patella, PatellaCart;
    JointBase Joint;

	public void build (String [] args) throws IOException {
		
		// set a gravity
		mech.setGravity (0, -9.81, 0);
        
		// Import rigid bodies
        Femur = importRigidBody("femur.obj", "Femur", 1800);
        TibiaFibula = importRigidBody("tibia.obj", "TibiaFibula", 1800);
        Patella = importRigidBody("patella.obj", "Patella", 1800);
        FemurCart = importRigidBody("femcart.obj", "FemurCart", 1100);
        TibiaCart = importRigidBody("tibcart.obj", "TibiaCart", 1100);
        Meniscus = importRigidBody("Meniscus.obj", "Meniscus", 1100);
        PatellaCart = importRigidBody("patcart.obj", "PatellaCart", 1100);
        
        // Set rigid body as non-dynamic
        TibiaFibula.setDynamic(false);
        
		addModel(mech);
		
		// set joint
        Joint = createJoint (Femur, TibiaFibula);
        
        // Set collision behaviors
//        setCollisionBehavior(Femur, FemurCart, 1e-6, 0.5);
//        setCollisionBehavior(TibiaFibula, TibiaCart, 1e-6, 0.5);
//        setCollisionBehavior(TibiaCart, Meniscus, 1e-6, 0.5);
//        setCollisionBehavior(FemurCart, Meniscus, 1e-6, 0.5);
//        setCollisionBehavior(TibiaFibula, Femur, 1e-6, 0.5);
//        setCollisionBehavior(Femur, TibiaFibula, 1e-6, 0.5);
        setCollisionBehavior(Femur, 1e-6, 0.5);
        setCollisionBehavior(TibiaFibula, 1e-6, 0.5);
        setCollisionBehavior(Patella, 1e-6, 0.5);
        setCollisionBehavior(FemurCart, 1e-6, 0.5);
        setCollisionBehavior(TibiaCart, 1e-6, 1);
        setCollisionBehavior(Meniscus, 1e-6, 0.5);
        setCollisionBehavior(PatellaCart, 1e-6, 0.5);
        
        // Enable collision force visualization
        setCollisionManager();
        
        // add a Input Probe
        createInputProbe ();
	}
	// import Rigid model
    private RigidBody importRigidBody(String filename, String name, double density) throws IOException {
        PolygonalMesh mesh = new PolygonalMesh(Modeldata + filename);
        RigidBody body = RigidBody.createFromMesh(name, mesh, density, 1);
        mech.addRigidBody(body);
        return body;
    }
	// set collision Behavior
    private void setCollisionBehavior(RigidBody body, double compliance, double damping) {
        CollisionBehavior behavior = new CollisionBehavior(true, 0.01);
        behavior.setCompliance(compliance);
        behavior.setDamping(damping);
        mech.setCollisionBehavior(body, Collidable.All, behavior);
	}
    
	// set collision Behavior
    private void setCollisionBehavior(RigidBody bodyA, RigidBody bodyB, double compliance, double damping) {
        CollisionBehavior behavior = new CollisionBehavior(true, 0.01);
        behavior.setCompliance(compliance);
        behavior.setDamping(damping);
        mech.setCollisionBehavior(bodyA, bodyB, behavior);
	}
    // set collision manager
    private void setCollisionManager() {
        CollisionManager cm = mech.getCollisionManager();
        cm.setDrawIntersectionPoints(true);
        cm.setDrawContactForces(false);
        cm.setDrawFrictionForces(false);
        cm.setContactForceLenScale(0.1);
        RenderProps.setVisible(cm, true);
        RenderProps.setSolidArrowLines(cm, 0.2, Color.RED);
        RenderProps.setSphericalPoints(cm, 1 , Color.GREEN);
    }
    // create a InputProbe
	private void createInputProbe () throws IOException {
        // set a FrameMarker for ForceProbe
        //FrameMarker mkrProbe = new FrameMarker (337, 1630, 823);
        //mkrProbe.setFrame(Femur);
        //mech.addFrameMarker(mkrProbe);
		FrameMarker mkrProbe = mech.addFrameMarkerWorld(Femur, new Point3d(337, 1630, 823));
        RenderProps.setSphericalPoints (mkrProbe, 4, Color.BLUE);
        // create a ForceProbe
		NumericInputProbe ForceProbe = 
				new NumericInputProbe (
						mech, "frameMarkers/0:externalForce",
						PathFinder.getSourceRelativePath(this, "ForceforFemur.txt"));
		ForceProbe.setName("force");
		addInputProbe (ForceProbe);
	}
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
        joint.setRoll(0);
        joint.setPitch(0);
        joint.setYaw(0);
        // lock a Joint
        joint.setPitchLocked(true);
        joint.setYawLocked(true);
        mech.addBodyConnector(joint);
        setJointCompliance(joint);
        setJointRenderProps(joint);
        JointControl(joint);
        return joint;
	}
	// set Compliance for Joint
    private void setJointCompliance(JointBase joint) {
        VectorNd comp = new VectorNd(joint.numConstraints());
        VectorNd damp = new VectorNd(joint.numConstraints());
        for (int i = 0; i < joint.numConstraints(); i++) {
            comp.set(i, 1e-9);
            damp.set(i, 0.1);
        }
        joint.setCompliance(comp);
        joint.setDamping(damp);
    }
 	// set RenderProps for joint
	private void setJointRenderProps (JointBase joint) {
		joint.setShaftLength (70);
		joint.setShaftRadius (1);
		joint.setAxisLength (50);
		joint.setDrawFrameC (AxisDrawStyle.ARROW);
		joint.setDrawFrameD (AxisDrawStyle.ARROW);
	}
	// set control for Joint
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