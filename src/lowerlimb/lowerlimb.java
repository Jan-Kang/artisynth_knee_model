package lowerlimb;

import java.awt.Color;
import java.io.IOException;
import java.util.List;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.SimpleAxialMuscle;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.ConnectableBody;
import artisynth.core.mechmodels.ContactData;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.GimbalJoint;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.util.PathFinder;

public class lowerlimb extends RootModel {
	// path of data
	String ModelData = PathFinder.getSourceRelativePath (this, "data/");	
	// create MECH
	MechModel mech = new MechModel ();
	// create FEM
    FemModel3d Femur;
    FemModel3d TiFi;
    FemModel3d collisionMeshleft;
    FemModel3d collisionMeshright;
    // create a Joint between Femur and TibiaFibula 
    JointBase JointFTF;
    // set collision behavior
    CollisionBehavior behav1, behav2, behav3;
    // set Collision Response
    CollisionResponse resp;
    
	@Override
	public void build(String[] args) throws IOException {
        // gravity
        mech.setGravity (0, 0, -9.81);
        // model
        Femur = importFemur ();
        TiFi = importTibiaFibula ();
        addModel (mech);
        
        // create a collision mesh
        collisionMeshleft = creatCollisionMeshleft(
        		collisionMeshleft, TiFi, 
        		6.0, 12.0, 16, 1, 1,
        		107.891323, -23.194814, -435.43464);
        collisionMeshright = creatCollisionMeshright(
        		collisionMeshright, TiFi,
        		3.0, 8.5, 16, 1, 1,
        		60.891323, -42.194814, -435.43464); 
        
        // femur 275  TiFi 6610
        // create the muscleleft
        FemNode3d node275 = Femur.getNode (275);
        FemNode3d node6610 = TiFi.getNode (6610);
        Muscle muscleleft = new Muscle ("musleft", 0);
        muscleleft.setPoints(node275, node6610);
        muscleleft.setMaterial(new SimpleAxialMuscle(1,0,0));        
        RenderProps.setSpindleLines (muscleleft, 2, Color.RED);
        mech.addAxialSpring (muscleleft);
        
        // femur 201 TiFi 6086
        // create the muscleright
        FemNode3d node201 = Femur.getNode(201);
        FemNode3d node6081 = TiFi.getNode(6086);
        Muscle muscleright = new Muscle ("musright", 0);
        muscleright.setPoints(node201, node6081);
        muscleright.setMaterial(new SimpleAxialMuscle(1,0,0));        
        RenderProps.setSpindleLines (muscleright, 2, Color.RED);
        mech.addAxialSpring (muscleright);
        
        // femur 298 2784 TiFi 6080 9235 6096
        // create the musclermid
        FemNode3d node298 = Femur.getNode(298);
        FemNode3d node2784 = Femur.getNode(2784);
        // FemNode3d node6096 = TiFi.getNode(6096);
        // FemNode3d node9235 = TiFi.getNode(9235);
        FemNode3d node6080 = TiFi.getNode(6080);
        MultiPointMuscle musclemid = new MultiPointMuscle("musmid");
        musclemid.addPoint (node298);
        musclemid.addPoint (node2784);
//        musclemid.addPoint (node9235);
//        musclemid.addPoint (node6096);
        musclemid.addPoint (node6080);
        musclemid.setMaterial (new SimpleAxialMuscle(1,0,0));    
        RenderProps.setSpindleLines (musclemid, 2, Color.RED);
        mech.addMultiPointSpring (musclemid);
        
        // joint
        JointFTF = createJoint (Femur, TiFi);
        JointFTF.setEnabled (true);
        // contact
        setCollisionBehavior (behav1, Femur, TiFi);
        setCollisionBehavior (behav2, Femur, collisionMeshleft);
        setCollisionBehavior (behav3, Femur, collisionMeshright);                      
	}	
		
	// import FEM model
	private FemModel3d importFemur () throws IOException {
		// import Femur
		FemModel3d Femur = null;
		Femur = new FemModel3d ("Femur");
		Femur = AnsysCdbReader.read (ModelData + "Femur.cdb");
		// set physical properties
		Femur.setDensity (1.9e-6);
		Femur.setMassDamping (0.01);
		Femur.setStiffnessDamping (0.02);
		Femur.setMaterial (new LinearMaterial (1e9, 0.3));
		Femur.setName ("Femur");
		if (Femur.isVolumeValid ())
			System.out.println ("Femur mesh valid.");
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
		mech.addModel (TibiaFibula);
		setFemRenderProps (TibiaFibula);
		return TibiaFibula;
	}
	
	// set FEM model RenderProps
	private void setFemRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering (SurfaceRender.Shaded);
		RenderProps.setSphericalPoints (fem, 0.6, Color.GREEN);
		RenderProps.setLineColor (fem, Color.darkGray);
		RenderProps.setFaceColor (fem, Color.LIGHT_GRAY);
	}
	
	// create a collision mesh right
	private FemModel3d creatCollisionMeshleft(
			FemModel3d collisionMesh, FemModel3d fem, 
			double l, double r, int nt, int nl, int nr,
			double x, double y, double z) {
		collisionMesh = FemFactory.createCylinder(collisionMesh, l, r, nt, nl, nr);
        mech.addModel(collisionMesh); 
		// set physical properties
        collisionMesh.setDensity (1.9e-6);
        collisionMesh.setMassDamping (0.01);
        collisionMesh.setStiffnessDamping (0.02);
        collisionMesh.setMaterial (new LinearMaterial (1e9, 0.3));
        collisionMesh.setName ("collisionMeshleft");
    	setCollisionMeshRenderProps(collisionMesh);
        // set new Position
        Vector3d offset = new Vector3d (x, y, z);
        for(FemNode3d node: collisionMesh.getNodes()) {
        	Point3d pos = new Point3d(node.getPosition());
        	pos.add(offset);
        	node.setPosition(pos);
            }
        // update property
        collisionMesh.updateSlavePos ();
        collisionMesh.invalidateStressAndStiffness ();
        // connect between collision and model
        Point3d nearestPoint = new Point3d ();
    	for (FemNode3d n: collisionMesh.getNodes ()) {
    		if (collisionMesh.isSurfaceNode(n)) {
    			if (n.getPosition ().z < z) {
    				FemElement3dBase nearestElem = fem.findNearestSurfaceElement (nearestPoint, n.getPosition ());
    				if (nearestElem != null) {
    					mech.attachPoint(n, nearestElem);
        				}
    				}
    			}
        	}
    	return collisionMesh;
	}
	
	// create a collision mesh right
	private FemModel3d creatCollisionMeshright(
			FemModel3d collisionMesh, FemModel3d fem, 
			double l, double r, int nt, int nl, int nr,
			double x, double y, double z) {
		collisionMesh = FemFactory.createCylinder(collisionMesh, l, r, nt, nl, nr);
        mech.addModel(collisionMesh);   
		// set physical properties
        collisionMesh.setDensity (1.9e-4);
        collisionMesh.setMassDamping (0.01);
        collisionMesh.setStiffnessDamping (0.02);
        collisionMesh.setMaterial (new LinearMaterial (1e9, 0.3));
        collisionMesh.setName ("collisionMeshright");
    	setCollisionMeshRenderProps(collisionMesh);
        // set new Position
        Vector3d offset = new Vector3d (x, y, z);
        for(FemNode3d node: collisionMesh.getNodes()) {
        	Point3d pos = new Point3d(node.getPosition());
        	pos.add(offset);
        	node.setPosition(pos);
            }
        // update property
        collisionMesh.updateSlavePos ();
        collisionMesh.invalidateStressAndStiffness ();
        // connect between collision and model
        Point3d nearestPoint = new Point3d ();
    	for (FemNode3d n: collisionMesh.getNodes ()) {
    		if (collisionMesh.isSurfaceNode(n)) {
    			if (n.getPosition ().z < z) {
    				FemElement3dBase nearestElem = fem.findNearestSurfaceElement (nearestPoint, n.getPosition ());
    				if (nearestElem != null) {
    					mech.attachPoint(n, nearestElem);
        				}
    				}
    			}
        	}
    	return collisionMesh;
	}

	// set collision mesh RenderProps
	private void setCollisionMeshRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering (SurfaceRender.Shaded);
		RenderProps.setLineColor (fem, Color.darkGray);
		RenderProps.setFaceColor (fem, Color.ORANGE);
	}
	
	
	// create joint
	private JointBase createJoint (ConnectableBody femur, ConnectableBody tifi) {
		// create a HingeJoint joint that connects the two fem
		Vector3d origin = new Vector3d (82.891323, -35.194814, -421.43464);
		RigidTransform3d TDW =
				new RigidTransform3d (origin.x, origin.y, origin.z);
		TDW.setRpyDeg (0, 90, -20);
		GimbalJoint joint = new GimbalJoint (femur, tifi, TDW);
	    // set joint ranges (in degrees)
		joint.setRollRange (-90, 0);
	    joint.setPitchRange (-5, 5);
	    joint.setYawRange (-5, 5);
	    /*
	    // set joint initial value
	    joint.setRoll (0);
	    joint.setPitch (0);
	    joint.setYaw (0);
	    */
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
	
	// Control Joint
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
		
	// set collision Behavior
	private void setCollisionBehavior (CollisionBehavior behav, FemModel3d fem1, FemModel3d fem2) {
        behav = mech.setCollisionBehavior (fem1, fem2, true, 0);
        behav.setCompliance (50);
        behav.setDamping (20);
        resp = mech.setCollisionResponse (fem1, fem2);
        setCollisionManager();
        addMonitor (new ContactMonitor());
	}
	
    // set collision manager
	private void setCollisionManager () {
		CollisionManager cm = mech.getCollisionManager();
        cm.setDrawContactForces (true);
        cm.setDrawFrictionForces (true);
        cm.setContactForceLenScale (1);
        RenderProps.setVisible (cm, true);
        RenderProps.setSolidArrowLines (cm, 0.2, Color.RED);
	}
	
	// set contact monitor
	private class ContactMonitor extends MonitorBase {
		public void apply (double t0, double t1) {
			List<ContactData> cdata = resp.getContactData();
			if (cdata.size() > 0) {
				System.out.println ("num contacts: "+ cdata.size() + ", time=" + t0);
				for (ContactData cd : cdata) {
					System.out.print (" pos:   " + cd.getPosition0().toString("%8.3f"));
					System.out.println (", force: " + cd.getContactForce().toString("%8.1f"));
				}
			}
		}
	}
	
	/*
	// calculates the model size
	private void ModelSize (FemModel3d fem) {
	    Point3d min = new Point3d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
	    Point3d max = new Point3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
	    for (FemNode3d node : fem.getNodes()) {
	        Point3d pos = node.getPosition();
	        if (pos.x > max.x) {max.x = pos.x;}
	        if (pos.y > max.y) {max.y = pos.y;}
	        if (pos.z > max.z) {max.z = pos.z;}
	        if (pos.x < min.x) {min.x = pos.x;}
	        if (pos.y < min.y) {min.y = pos.y;}
	        if (pos.z < min.z) {min.z = pos.z;}
	    }
		double width = max.x - min.x;
	    double height = max.y - min.y;
	    double depth = max.z - min.z;
	    System.out.println("Model Size:");
	    System.out.println("Min: " + min.toString());
	    System.out.println("Max: " + max.toString());
	    System.out.println("Width, Height, Depth: " + width + ", " + height + ", " + depth);
	}
	*/
}
