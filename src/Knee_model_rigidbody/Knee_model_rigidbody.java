package Knee_model_rigidbody;

import java.awt.Color;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
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
import artisynth.core.mechmodels.PointFrameAttachment;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.renderables.ColorBar;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

public class Knee_model_rigidbody extends RootModel {

	String Modeldata = maspack.util.PathFinder.getSourceRelativePath(this, "data/");
	MechModel mech = new MechModel();
	RigidBody Femur, Meniscus, TibiaFibula, TibiaCart, Patella, PatellaCart;
	FemModel3d FemurCart;
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
		TibiaCart = importRigidBody("tibcart.obj", "TibiaCart", 2.1e-5);
		Meniscus = importRigidBody("Meniscus.obj", "Meniscus", 2.1e-5);
		PatellaCart = importRigidBody("patcart.obj", "PatellaCart", 2.1e-5);

		// Set rigid body as non-dynamic
		TibiaFibula.setDynamic(false);

		// import FEM model
		FemurCart = importFemModel("mesh_FemurCart.cdb", "FemurCart", 2.1e-5, 0.02, 1e-6, new LinearMaterial(1, 0.45));

		// connecting FEM Model to Rigid Body model
		for (FemNode3d n : FemurCart.getNodes()) {
			if (FemurCart.isSurfaceNode(n)) {
				if (n.getPosition().y > 1390) {
					RenderProps.setVisible(n, true);
					RenderProps.setSphericalPoints(n, 0.1, Color.red);
					mech.attachPoint(n, Femur);
				}
			}
		}

		// set joint
		Joint = createJoint(Femur, TibiaFibula);

		// Set collision behaviors
		setCollisionBehavior(Femur, 0.01, 1e-6, 1e6);
		setCollisionBehavior(TibiaFibula, 0.01, 1e-6, 1e6);
		setCollisionBehavior(Patella, 0.01, 1e-6, 1e6);
		setCollisionBehavior(FemurCart, 0.01, 1e-6, 0.5);
		setCollisionBehavior(TibiaCart, 0.01, 1e-6, 1e6);
		setCollisionBehavior(Meniscus, 0.01, 1e-6, 1e6);
		setCollisionBehavior(PatellaCart, 0.01, 1e-6, 0.5);

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

	// import FEM Model
	private FemModel3d importFemModel(String filename, String name, double density, double massDamping,
			double stiffnessDamping, FemMaterial material) throws IOException {
		FemModel3d femModel = AnsysCdbReader.read(Modeldata + filename);
		femModel.setDensity(density);
		femModel.setMassDamping(massDamping);
		femModel.setStiffnessDamping(stiffnessDamping);
		femModel.setMaterial(material);
		femModel.setName(name);
		mech.addModel(femModel);
		setFemRenderProps(femModel);
		return femModel;
	}

	// set FEM model render properties
	private void setFemRenderProps(FemModel3d fem) {
		// fem.setSurfaceRendering(SurfaceRender.Stress);
		// fem.setStressPlotRanging(Ranging.Auto);
		// RenderProps.setVisible(fem.getNodes(), false);
		// RenderProps.setVisible(fem.getElements(), false);
		// RenderProps.setAlpha(fem, 1.0);
		fem.setSurfaceRendering(SurfaceRender.Shaded);
		// RenderProps.setFaceColor (fem, Color.GRAY);
		RenderProps.setLineColor(fem, Color.DARK_GRAY);
		// RenderProps.setSphericalPoints (fem, 0.2, Color.CYAN);

		// create a color bar
		/*
		 * ColorBar cbar = new ColorBar(); 
		 * cbar.setName("colorBar");
		 * cbar.setNumberFormat("%.2f"); 
		 * cbar.populateLabels(0.0, 0.1, 10);
		 * cbar.setLocation(-100, 0.1, 20, 0.8); 
		 * addRenderable(cbar);
		*/
	}
	
	/*
	public void prerender(RenderList list) {
		super.prerender(list);
		ColorBar cbar = (ColorBar) (renderables().get("colorBar"));
		List<FemModel3d> femModels = Arrays.asList(FemurCart);
		for (FemModel3d fem : femModels) {
			cbar.setColorMap(fem.getColorMap());
			DoubleInterval range = fem.getStressPlotRange();
			cbar.updateLabels(range.getLowerBound(), range.getUpperBound());
		}
	}
	*/

	// set collision Behavior
	private void setCollisionBehavior(Collidable Model, double mu, double compliance, double damping) {
		CollisionBehavior behavior = new CollisionBehavior(true, mu);
		behavior.setCompliance(compliance);
		behavior.setDamping(damping);
		mech.setCollisionBehavior(Model, Collidable.All, behavior);
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
		RenderProps.setSphericalPoints(cm, 0.5, Color.GREEN);
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