package Knee_model_rigidbody;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
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
	MechModel mech = new MechModel();
	RigidBody Femur, Meniscus, TibiaFibula, Patella, PatellaCart;
	FemModel3d FemurCart, TibiaCart;
	JointBase Joint;

	public void build(String[] args) throws IOException {

		addModel(mech);

		// set a gravity
		mech.setGravity(0, -9.81, 0);

		// import rigid bodies
		Femur = importRigidBody("femur.obj", "Femur", 2.1e-5);
		TibiaFibula = importRigidBody("tibia.obj", "TibiaFibula", 2.1e-5);
		Patella = importRigidBody("patella.obj", "Patella", 2.1e-5);
		// FemurCart = importRigidBody("femcart.obj", "FemurCart", 2.1e-5);
		// TibiaCart = importRigidBody("tibcart.obj", "TibiaCart", 2.1e-5);
		Meniscus = importRigidBody("Meniscus.obj", "Meniscus", 2.1e-5);
		// PatellaCart = importRigidBody("patcart.obj", "PatellaCart", 2.1e-5);

		// Set rigid body as non-dynamic
		TibiaFibula.setDynamic(false);

		// import fem model
		FemurCart = importFemModel("mesh_FemurCart.cdb", "FemurCart", 2.1e-10, 0.02, 1e-6,
				new LinearMaterial(1, 0.3));

		// set joint
		Joint = createJoint(Femur, TibiaFibula);

		// Set collision behaviors
		setCollisionBehavior(Femur, 1e-6, 1e6);
		setCollisionBehavior(TibiaFibula, 1e-6, 1e6);
		setCollisionBehavior(Patella, 1e-6, 1e6);
		// setCollisionBehavior(FemurCart, 1e-6, 0.5);
		// setCollisionBehavior(TibiaCart, 1e-6, 1e6);
		setCollisionBehavior(Meniscus, 1e-6, 1e6);
		// setCollisionBehavior(PatellaCart, 1e-6, 0.5);

		// Enable collision force visualization
		setCollisionManager();

		// add a Input Probe
		createInputProbe();
		
		// set a stop time
		addBreakPoint(5.0);
	}

	// import Rigid model
	private RigidBody importRigidBody(String filename, String name, double density) throws IOException {
		PolygonalMesh mesh = new PolygonalMesh(Modeldata + filename);
		RigidBody body = RigidBody.createFromMesh(name, mesh, density, 1);
		mech.addRigidBody(body);
		return body;
	}

	// import Fem Model
	private FemModel3d importFemModel(String filename, String name, double density, double massDamping,
			double stiffnessDamping, FemMaterial material) throws IOException {
		// Import model
		FemModel3d femModel = AnsysCdbReader.read(Modeldata + filename);
		// Set properties
		femModel.setDensity(density);
		femModel.setMassDamping(massDamping);
		femModel.setStiffnessDamping(stiffnessDamping);
		femModel.setMaterial(material);
		femModel.setName(name);
		// Add model to the mechanical system
		mech.addModel(femModel);
		setFemRenderProps(femModel);
		return femModel;
	}

	// set FEM model render properties
	private void setFemRenderProps(FemModel3d fem) {
		// fem.setSurfaceRendering(SurfaceRender.Stress);
		// fem.setStressPlotRanging(Ranging.Auto);
		RenderProps.setFaceColor (fem, Color.LIGHT_GRAY);
		RenderProps.setAlpha(fem, 1.0);
		RenderProps.setVisible(fem.getNodes(), false);
		RenderProps.setVisible(fem.getElements(), true);
		RenderProps.setLineColor(fem, Color.darkGray);
		// RenderProps.setSphericalPoints (fem, 0.2, Color.CYAN);
	}

	// set collision Behavior
	private void setCollisionBehavior(RigidBody body, double compliance, double damping) {
		CollisionBehavior behavior = new CollisionBehavior(true, 0.05);
		behavior.setCompliance(compliance);
		behavior.setDamping(damping);
		mech.setCollisionBehavior(body, Collidable.All, behavior);
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
		RenderProps.setSphericalPoints(cm, 1, Color.GREEN);
	}

	// create a InputProbe
	private void createInputProbe() throws IOException {
		// set a FrameMarker for ForceProbe
		FrameMarker mkrProbe = mech.addFrameMarkerWorld(Femur, new Point3d(337, 1630, 823));
		RenderProps.setSphericalPoints(mkrProbe, 4, Color.BLUE);
		// create a ForceProbe
		NumericInputProbe ForceProbe = new NumericInputProbe(mech, "frameMarkers/0:externalForce",
				PathFinder.getSourceRelativePath(this, "ForceforFemur.txt"));
		ForceProbe.setName("force");
		addInputProbe(ForceProbe);
	}

	// create joint
	private JointBase createJoint(ConnectableBody femur, ConnectableBody tifi) {
		Vector3d origin = new Vector3d(360, 1380, 840);
		RigidTransform3d TDW = new RigidTransform3d(origin.x, origin.y, origin.z);
		TDW.setRpyDeg(0, 90, 0);
		GimbalJoint joint = new GimbalJoint(femur, tifi, TDW);
		// set joint ranges (in degrees)
		joint.setRollRange(-90, 0);
		joint.setPitchRange(-5, 5);
		joint.setYawRange(-5, 5);
		// set joint initial value
		joint.setRoll(0);
		joint.setPitch(0);
		joint.setYaw(0);
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
			damp.set(i, 1e9);
		}
		joint.setCompliance(comp);
		joint.setDamping(damp);
	}

	// set RenderProps for joint
	private void setJointRenderProps(JointBase joint) {
		joint.setShaftLength(70);
		joint.setShaftRadius(1);
		joint.setAxisLength(50);
		joint.setDrawFrameC(AxisDrawStyle.ARROW);
		joint.setDrawFrameD(AxisDrawStyle.ARROW);
	}

	// set control for Joint
	private void JointControl(JointBase joint) {
		ControlPanel panel = new ControlPanel();
		panel.addWidget(joint, "roll");
		panel.addWidget(joint, "pitch");
		panel.addWidget(joint, "yaw");
		panel.addWidget(joint, "rollRange");
		panel.addWidget(joint, "pitchRange");
		panel.addWidget(joint, "yawRange");
		panel.addWidget(joint, "rollLocked");
		panel.addWidget(joint, "pitchLocked");
		panel.addWidget(joint, "yawLocked");
		panel.addWidget(joint, "drawFrameC");
		panel.addWidget(joint, "drawFrameD");
		panel.addWidget(joint, "axisLength");
		panel.addWidget(joint, "jointRadius");
		panel.addWidget(joint, "linearCompliance");
		panel.addWidget(joint, "rotaryCompliance");
		panel.addWidget(joint, "compliance");
		panel.addWidget(joint, "damping");
		addControlPanel(panel);
	}
}