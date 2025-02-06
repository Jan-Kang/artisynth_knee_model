package Knee_model;

import java.awt.Color;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.LinearAxialMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.ConnectableBody;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.GimbalJoint;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.PointFrameAttachment;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.renderables.ColorBar;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

public class Knee_model extends RootModel {
	// path of data
	String Modeldata = maspack.util.PathFinder.getSourceRelativePath(this, "data/");
	// create MECH
	MechModel mech = new MechModel ();
	// create Rigid
	RigidBody FemurRigid, TibiaFibulaRigid;
	// create FEM
	FemModel3d Femur, FemurCart, 
			   Mensicus, 
			   TibiaCart, TibiaFibula,
			   Patella, PatellaCart;
	// create collision behaviors
    CollisionBehavior behav0, behav1, behav2, behav3, behav4, behav5, behav6, behav7;
    // create collision response
    CollisionResponse resp;
    // create joint
    JointBase Joint;
	
	@Override
	public void build (String [] args) throws IOException {
		// set a gravity
		mech.setGravity (0, -9.81, 0);
		// import rigid model
	    FemurRigid = importFemurRigid ();
	    TibiaFibulaRigid = importTibiaFibulaRigid ();
	    TibiaFibulaRigid.setDynamic(false);
	    // import FEM model
		Femur = importFemur ();
		FemurCart = importFemurCart ();
		Mensicus = importMensicus ();
		TibiaCart = importTibiaCart ();
		TibiaFibula = importTibiaFibula ();
		Patella = importPatella ();
		PatellaCart = importPatellaCart ();
		addModel (mech);
		// create a color bar
		ColorBar cbar = new ColorBar ();
		cbar.setName ("colorBar");
	    cbar.setNumberFormat ("%.2f");
	    cbar.populateLabels (0.0, 0.1, 10);
	    cbar.setLocation (-100, 0.1, 20, 0.8);
	    addRenderable (cbar);
		// attachment femur
    	for (FemNode3d femurNode: Femur.getNodes ()) {
    		if (Femur.isSurfaceNode(femurNode)) {
    			if (femurNode.getPosition ().y > 1438 ) {
						// RenderProps.setVisible(femurNode, true);
						// RenderProps.setSphericalPoints(femurNode, 0.1, Color.red);
    					mech.addAttachment (new PointFrameAttachment (FemurRigid, femurNode));
    				}
    			}
        	}
		// attachment TiFi
		for (FemNode3d TiFiNode : TibiaFibula.getNodes ()) {
    		if (TibiaFibula.isSurfaceNode(TiFiNode)) {
    			if (TiFiNode.getPosition().y < 1290) {
    					// RenderProps.setVisible(TiFiNode, true);
    					// RenderProps.setSphericalPoints(TiFiNode, 0.1, Color.red);
						mech.addAttachment (new PointFrameAttachment (TibiaFibulaRigid, TiFiNode));
				}
    		}
		}
		// set contacts between models
		setCollisionBehavior (behav0, Femur, TibiaFibula);
        setCollisionBehavior (behav1, Femur, FemurCart);
        setCollisionBehavior (behav2, TibiaFibula, TibiaCart);
        setCollisionBehavior (behav3, FemurCart, TibiaCart);
        setCollisionBehavior (behav4, FemurCart, Patella);
        setCollisionBehavior (behav5, Patella, PatellaCart);
        setCollisionBehavior (behav6, Mensicus, FemurCart);
        setCollisionBehavior (behav7, Mensicus, TibiaCart);
        // set a joint
        Joint = createJoint (FemurRigid, TibiaFibulaRigid);
        Joint.setEnabled (true);
		// set ligaments
		// MCL Medial Collateral Ligament
        FemNode3d node4236 = Femur.getNode (4236);
        FemNode3d node31321 = Femur.getNode (31321);
        FemNode3d node17254 = Mensicus.getNode(17254);
        FemNode3d node8749 = TibiaFibula.getNode (8749);
        FemNode3d node23525 = TibiaFibula.getNode (23525);
        FemNode3d node9417 = TibiaFibula.getNode (9417);
        MultiPointSpring  MCL = new MultiPointSpring ("MCL");
        MCL.setMaterial (new LinearAxialMaterial (2e-6, 0.02)); //setStiffness setDamping
        MCL.addPoint (node4236);
        MCL.addPoint (node31321);
        MCL.addPoint (node17254);
        MCL.addPoint (node8749);
        MCL.addPoint (node23525);
        MCL.addPoint (node9417);
        RenderProps.setCylindricalLines(MCL, 2, Color.cyan);
        mech.addMultiPointSpring (MCL);
        // LCL Lateral Collateral Ligament
        FemNode3d node2015 = Femur.getNode (2015);
        FemNode3d node294 = TibiaFibula.getNode (294);
        MultiPointSpring  LCL = new MultiPointSpring ("LCL");
        LCL.setMaterial (new LinearAxialMaterial (2e-6, 0.02)); //setStiffness setDamping
        LCL.addPoint (node2015);
        LCL.addPoint (node294);
        RenderProps.setSpindleLines (LCL, 2, Color.cyan);
        mech.addMultiPointSpring (LCL);
        // ACL Anterior Cruciate Ligament
        FemNode3d node389 = Femur.getNode (389);
        FemNode3d node8152 = TibiaFibula.getNode (8152);
        MultiPointSpring  ACL = new MultiPointSpring ("ACL");
        ACL.setMaterial (new LinearAxialMaterial (2e-6, 0.03)); //setStiffness setDamping
        ACL.addPoint (node389);
        ACL.addPoint (node8152);
        RenderProps.setSpindleLines (ACL, 2, Color.cyan);
        mech.addMultiPointSpring (ACL);
        // PCL Posterior Cruciate Ligament
        FemNode3d node1113 = Femur.getNode (1113);
        FemNode3d node8512 = TibiaFibula.getNode (8512);
        MultiPointSpring  PCL = new MultiPointSpring ("PCL");
        PCL.setMaterial (new LinearAxialMaterial (2e-6, 0.03)); //setStiffness setDamping
        PCL.addPoint (node1113);
        PCL.addPoint (node8512);
        RenderProps.setSpindleLines (PCL, 2, Color.cyan);
        mech.addMultiPointSpring (PCL);
        // POL Posterior Oblique Ligament
        FemNode3d node3768 = Femur.getNode (3768);
        FemNode3d node3929 = Femur.getNode (3929);
        FemNode3d node21042 = Mensicus.getNode(21042);
        FemNode3d node8691 = TibiaFibula.getNode(8691);
        FemNode3d node8809 = TibiaFibula.getNode (8809);
        MultiPointSpring  POL = new MultiPointSpring ("POL");
        POL.setMaterial (new LinearAxialMaterial (2e-6, 0.02)); //setStiffness setDamping
        POL.addPoint (node3768);
        POL.addPoint(node3929);
        POL.addPoint(node21042);
        POL.addPoint(node8691);
        POL.addPoint (node8809);
        RenderProps.setCylindricalLines (POL, 2, Color.cyan);
        mech.addMultiPointSpring (POL);
        // PL Patellar Ligament
        FemNode3d node1914 = Patella.getNode (1914);
        FemNode3d node9174 = TibiaFibula.getNode (9174);
        MultiPointSpring  PL = new MultiPointSpring ("PL");
        PL.setMaterial (new LinearAxialMaterial (2e-6, 0.02)); //setStiffness setDamping
        PL.addPoint (node1914);
        PL.addPoint (node9174);
        RenderProps.setSpindleLines (PL, 2, Color.cyan);
        mech.addMultiPointSpring (PL);
        // set M. quadriceps femoris
        FrameMarker femurmkr = new FrameMarker (348, 1497, 854);
        femurmkr.setFrame (FemurRigid);
        mech.addFrameMarker (femurmkr);
        FemNode3d node2255 = Patella.getNode (2255);
        FemNode3d node2213 = Patella.getNode (2213);
        FemNode3d node1660 = Patella.getNode (1660);
        FemNode3d node9295 = TibiaFibula.getNode (9295);
        MultiPointMuscle MQF = new MultiPointMuscle ("MQF");
        MQF.setMaterial (new LinearAxialMaterial (1e-6, 0.02)); //setStiffness setDamping
        MQF.addPoint (femurmkr);
        MQF.addPoint (node2255);
        MQF.addPoint (node2213);
        MQF.addPoint (node1660);
        // MQF.addPoint (node9295);
        RenderProps.setCylindricalLines (MQF, 2, Color.red);
        mech.addMultiPointSpring(MQF);
        
        // set a FrameMarker for Probe
        FrameMarker mkrProbe = new FrameMarker (337, 1630, 823);
        mkrProbe.setFrame(FemurRigid);
        mech.addFrameMarker(mkrProbe);
        RenderProps.setSphericalPoints (mkrProbe, 6, Color.BLUE);
        
        createInputProbe ();
        addBreakPoint(2.0);
	}
	
	private void createInputProbe () throws IOException {
		NumericInputProbe ForceProbe = 
				new NumericInputProbe (
						mech, "frameMarkers/1:force",
						PathFinder.getSourceRelativePath(this, "ForceforFemur.txt"));
		ForceProbe.setName("force");
		addInputProbe (ForceProbe);
	}
	
	// import Rigid model
	private RigidBody importFemurRigid () throws IOException {
		PolygonalMesh meshFemur = null;
		meshFemur = new PolygonalMesh (Modeldata + "mesh_femur_rigid.obj");
		RigidBody FemurRigid = 
				RigidBody.createFromMesh("FemurRigid", meshFemur, 1800, 1);
		mech.addRigidBody(FemurRigid);
		return FemurRigid;
	}	
	private RigidBody importTibiaFibulaRigid () throws IOException {
		PolygonalMesh meshTibiaFibulaRigid = null;
		meshTibiaFibulaRigid = new PolygonalMesh (Modeldata + "mesh_TibiaFibula_rigid.obj");
		RigidBody TibiaFibulaRigid = 
				RigidBody.createFromMesh("TibiaFibulaRigid", meshTibiaFibulaRigid, 1800, 1);
		mech.addRigidBody(TibiaFibulaRigid);
		return TibiaFibulaRigid;
	}
	// import FEM model
	private FemModel3d importFemur () throws IOException {
		// import model
		FemModel3d Femur = null;
		Femur = new FemModel3d ("Femur");
		Femur = AnsysCdbReader.read (Modeldata + "mesh_Femur_part.cdb");
		// set physical properties
		Femur.setDensity (1800); 
		Femur.setMassDamping (0.02);
		Femur.setStiffnessDamping (1e-6);
		Femur.setMaterial (new LinearMaterial (18e9, 0.3));
		Femur.setName ("Femur");
		if (Femur.isVolumeValid ())
			System.out.println ("Femur mesh valid.");
		mech.addModel (Femur);
		setFemRenderProps (Femur);
		return Femur;
	}
	private FemModel3d importFemurCart () throws IOException {
		// import model
		FemModel3d FemurCart = null;
		FemurCart = new FemModel3d ("FemurCart");
		FemurCart = AnsysCdbReader.read (Modeldata + "mesh_FemurCart.cdb");
		// set physical properties
		FemurCart.setDensity (1100);
		FemurCart.setMassDamping (0.02);
		FemurCart.setStiffnessDamping (1e-6);
		FemurCart.setMaterial (new LinearMaterial (12e6, 0.45));
		FemurCart.setName ("FemurCart");
		if (FemurCart.isVolumeValid ())
			System.out.println ("FemurCart mesh valid.");
		mech.addModel (FemurCart);
		setFemRenderProps (FemurCart);
		return FemurCart;
	}
	private FemModel3d importMensicus () throws IOException {
		// import model
		FemModel3d Mensicus = null;
		Mensicus = new FemModel3d ("Mensicus");
		Mensicus = AnsysCdbReader.read (Modeldata + "mesh_Mensicus.cdb");
		// set physical properties
		Mensicus.setDensity (1200);
		Mensicus.setMassDamping (0.03);
		Mensicus.setStiffnessDamping (5e-6);
		Mensicus.setMaterial (new LinearMaterial (120e6, 0.2));
		Mensicus.setName ("Mensicus");
		if (Mensicus.isVolumeValid ())
			System.out.println ("Mensicus mesh valid.");
		mech.addModel (Mensicus);
		setFemRenderProps (Mensicus);
		return Mensicus;
	}
	private FemModel3d importTibiaCart () throws IOException {
		// import model
		FemModel3d TibiaCart = null;
		TibiaCart = new FemModel3d ("TibiaCart");
		TibiaCart = AnsysCdbReader.read (Modeldata + "mesh_TibiaCart.cdb");
		// set physical properties
		TibiaCart.setDensity (1100);
		TibiaCart.setMassDamping (0.02);
		TibiaCart.setStiffnessDamping (1e-6);
		TibiaCart.setMaterial (new LinearMaterial (12e6, 0.45));
		TibiaCart.setName ("TibiaCart");
		if (TibiaCart.isVolumeValid ())
			System.out.println ("TibiaCart mesh valid.");
		mech.addModel (TibiaCart);
		setFemRenderProps (TibiaCart);
		return TibiaCart;
	}
	private FemModel3d importTibiaFibula () throws IOException {
		// import model
		FemModel3d TibiaFibula = null;
		TibiaFibula = new FemModel3d ("TibiaFibula");
		TibiaFibula = AnsysCdbReader.read (Modeldata + "mesh_TibiaFibula_part.cdb");
		// set physical properties
		TibiaFibula.setDensity (1800); 
		TibiaFibula.setMassDamping (0.02);
		TibiaFibula.setStiffnessDamping (1e-6);
		TibiaFibula.setMaterial (new LinearMaterial (18e9, 0.3));
		TibiaFibula.setName ("TibiaFibula");
		if (TibiaFibula.isVolumeValid ())
			System.out.println ("TibiaFibula mesh valid.");
		mech.addModel (TibiaFibula);
		setFemRenderProps (TibiaFibula);
		return TibiaFibula;
	}
	private FemModel3d importPatella () throws IOException {
		// import model
		FemModel3d Patella = null;
		Patella = new FemModel3d ("Patella");
		Patella = AnsysCdbReader.read (Modeldata + "mesh_Patella.cdb");
		// set physical properties
		Patella.setDensity (1800); 
		Patella.setMassDamping (0.02);
		Patella.setStiffnessDamping (1e-6);
		Patella.setMaterial (new LinearMaterial (18e9, 0.3));
		Patella.setName ("Patella");
		if (Patella.isVolumeValid ())
			System.out.println ("Patella mesh valid.");
		mech.addModel (Patella);
		setFemRenderProps (Patella);
		return Patella;
	}
	private FemModel3d importPatellaCart () throws IOException {
		// import model
		FemModel3d PatellaCart = null;
		PatellaCart = new FemModel3d ("PatellaCart");
		PatellaCart = AnsysCdbReader.read (Modeldata + "mesh_PatellaCart.cdb");
		// set physical properties
		PatellaCart.setDensity (1100);
		PatellaCart.setMassDamping (0.02);
		PatellaCart.setStiffnessDamping (1e-6);
		PatellaCart.setMaterial (new LinearMaterial (12e6, 0.45));
		PatellaCart.setName ("PatellaCart");
		if (PatellaCart.isVolumeValid ())
			System.out.println ("PatellaCart mesh valid.");
		mech.addModel (PatellaCart);
		setFemRenderProps (PatellaCart);
		return PatellaCart;
	}
	// set FEM model render properties
	private void setFemRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering (SurfaceRender.Stress);
		fem.setStressPlotRanging (Ranging.Auto);
		// RenderProps.setFaceColor (fem, Color.LIGHT_GRAY);
		RenderProps.setAlpha (fem, 1.0);
		RenderProps.setVisible (fem.getNodes(), false);
		RenderProps.setVisible (fem.getElements(), true);
		RenderProps.setLineColor (fem, Color.darkGray);
		// RenderProps.setSphericalPoints (fem, 0.2, Color.CYAN);
	}
	public void prerender (RenderList list) {
		super.prerender (list);
		ColorBar cbar = (ColorBar) (renderables().get("colorBar"));
		List<FemModel3d> femModels = Arrays.asList(Femur, FemurCart, Mensicus, TibiaCart, TibiaFibula, Patella, PatellaCart);
		for (FemModel3d fem : femModels) {
			cbar.setColorMap (fem.getColorMap());
			DoubleInterval range = fem.getStressPlotRange ();
			cbar.updateLabels (range.getLowerBound(), range.getUpperBound());
		}
	}
	// set collision Behavior
	private void setCollisionBehavior (CollisionBehavior behav, FemModel3d fem1, FemModel3d fem2) {
        behav = mech.setCollisionBehavior (fem1, fem2, true, 0);
        behav.setCompliance (1e-6);
        behav.setDamping (50);
        resp = mech.setCollisionResponse (fem1, fem2);
        // setCollisionManager ();
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