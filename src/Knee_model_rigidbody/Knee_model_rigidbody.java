package Knee_model_rigidbody;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemCutPlane;
import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.fields.ScalarElementField;
import artisynth.core.fields.ScalarFemField;
import artisynth.core.fields.ScalarNodalField;
import artisynth.core.fields.VectorNodalField;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.Blankevoort1991AxialLigament;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.SimpleAxialMuscle;
import artisynth.core.mechmodels.AxialSpring;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.ConnectableBody;
import artisynth.core.mechmodels.ContactData;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.GimbalJoint;
import artisynth.core.mechmodels.HingeJoint;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.Muscle;
import artisynth.core.mechmodels.PointFrameAttachment;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.FieldComponent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.renderables.ColorBar;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.render.Renderer.PointStyle;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

public class Knee_model_rigidbody extends RootModel {

	String Modeldata = maspack.util.PathFinder.getSourceRelativePath(this, "data/");
	MechModel mech = new MechModel();
	RigidBody Femur, Meniscus, TibiaFibula, TibiaCart, Patella, PatellaCart;
	FemModel3d meshFemurCart;
	JointBase Joint;
	CollisionResponse resp;


	public void build(String[] args) throws IOException {

		addModel(mech);

		// set a gravity
		mech.setGravity(0, -9.81, 0);
		
		// import rigid bodies
		Femur = importRigidBody("femur.obj", "Femur", 1.3e-6);
		TibiaFibula = importRigidBody("tibia.obj", "TibiaFibula", 1.3e-6);
		Patella = importRigidBody("patella.obj", "Patella", 1.3e-6);
		TibiaCart = importRigidBody("tibcart.obj", "TibiaCart", 1.15e-6);
		PatellaCart = importRigidBody("patcart.obj", "PatellaCart", 1.15e-6);
		Meniscus = importRigidBody("Meniscus.obj", "Meniscus", 1.1e-6);

		// import FEM models
		meshFemurCart = importFemModel("mesh_FemurCart.cdb", "FemurCart", 
				1.15e-6, // density in kg/mm³
				0.02, // mass damping
				50, // stiffness damping
				new LinearMaterial(10, 0.45));
		
		// import Ligaments
		addLigaments(mech, Femur, TibiaFibula, Meniscus, Patella);
		
		// set non-dynamic
		Femur.setDynamic(false);
		TibiaFibula.setDynamic(true);
		
		// connecting FEM Model to Rigid Body model
		PolygonalMesh surface = Femur.getSurfaceMesh();
		double tol = 1;
		for (FemNode3d n : meshFemurCart.getNodes()) {
			if (meshFemurCart.isSurfaceNode(n)) {
				Point3d point = n.getPosition();
				if (surface.distanceToPoint(point) < tol) {
//					RenderProps.setVisible(n, true);
//					RenderProps.setSphericalPoints(n, 0.1, Color.red);
					mech.attachPoint(n, Femur);
				}
			}
		}
		
		// connecting between Rigid Body models 		
		mech.attachFrame(Meniscus, TibiaCart);
		mech.attachFrame(TibiaCart, TibiaFibula);
		mech.attachFrame(PatellaCart, Patella);
		
		// set joint
		Joint = createJoint(Femur, TibiaFibula);

		// set collision behaviors
		setCollisionBehavior(Femur, PatellaCart, 0, 1e-6, 1e6);
		setCollisionBehavior(meshFemurCart, PatellaCart, 0, 1e-6, 1e6);
		setCollisionBehavior(meshFemurCart, Meniscus, 0, 1e-6, 1e6);
		setCollisionBehavior(meshFemurCart, TibiaCart, 0, 1e-6, 1e6);
		
		// set a collision response
		resp = mech.setCollisionResponse(meshFemurCart, Meniscus);
		addMonitor (new ContactMonitor());

		// enable collision force visualization
		// setCollisionManager();

		addProbe();

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
		femModel.setComputeNodalStress(true);
		return femModel;
	}

	// set FEM model render properties
	private void setFemRenderProps(FemModel3d fem) {
		fem.setSurfaceRendering(SurfaceRender.MAPStress);
		// fem.setStressPlotRanging(Ranging.Auto);
		// RenderProps.setVisible(fem.getNodes(), false);
		// RenderProps.setVisible(fem.getElements(), false);
		// RenderProps.setAlpha(fem, 1.0);
		// fem.setSurfaceRendering(SurfaceRender.Shaded);
		// RenderProps.setFaceColor (fem, Color.GRAY);
		// RenderProps.setLineColor(fem, Color.DARK_GRAY);
		// RenderProps.setSphericalPoints (fem, 0.2, Color.CYAN);

		// create a color bar
		ColorBar cbar = new ColorBar();
		cbar.setName("colorBar");
		cbar.setNumberFormat("%.2f");
		cbar.populateLabels(0.0, 0.1, 10);
		cbar.setLocation(-100, 0.1, 20, 0.8);
		addRenderable(cbar);

	}

	private double minValue;
	private double maxValue;

	public void prerender(RenderList list) {
		super.prerender(list);
		addScalarField();
		ColorBar cbar = (ColorBar) (renderables().get("colorBar"));
		List<FemModel3d> femModels = Arrays.asList(meshFemurCart);
		for (FemModel3d fem : femModels) {
			cbar.setColorMap(fem.getColorMap());
			cbar.updateLabels(minValue, maxValue);
		}
	}

	private void addScalarField() {
		ScalarNodalField field = new ScalarNodalField(meshFemurCart);
		meshFemurCart.addField(field);
		minValue = Double.MAX_VALUE;
		maxValue = Double.MIN_VALUE;
		for (FemNode3d n : meshFemurCart.getNodes()) {
			if (meshFemurCart.isSurfaceNode(n)) {
				double value = n.getMAPStress();
				if (value < minValue) {
					minValue = value;
				}
				if (value > maxValue) {
					maxValue = value;
				}
				System.out.print(n.getStress());
				System.out.print(n.getDisplacement());
				field.setValue(n, value);
				// n.getStress();
				// n.getDistance();
			}
		} 
 
		
		// field.setVisualization(ScalarNodalField.Visualization.POINT);
		// RenderProps.setPointRadius(field, 0.2);
		// RenderProps.setPointStyle(field, PointStyle.SPHERE);
	}

	// add Ligaments
	private void addLigaments(MechModel mech, RigidBody femur, RigidBody tibiaFibula, RigidBody meniscus, RigidBody patella) {
		Object[][] ligaments = {
		    {"ALL",   femur, new Point3d(325, 1377, 842), tibiaFibula, new Point3d(324, 1352, 844),  795, 23.5, 0.00, meniscus, new Point3d(324, 1362, 843)},	        
		    {"aACL",  femur, new Point3d(356, 1384, 834), tibiaFibula, new Point3d(362, 1357, 853), 6200, 32.3, 0.00},
	        {"pACL",  femur, new Point3d(356, 1379, 832), tibiaFibula, new Point3d(358, 1357, 850), 3400, 26.6, 0.00},
	        {"aLCL",  femur, new Point3d(326, 1381, 841), tibiaFibula, new Point3d(311, 1328, 829), 2000, 55.8, 0.00},
	        {"mLCL",  femur, new Point3d(327, 1381, 837), tibiaFibula, new Point3d(311, 1328, 825), 2000, 53.2, 0.00},
	        {"pLCL",  femur, new Point3d(329, 1380, 833), tibiaFibula, new Point3d(313, 1331, 823), 2000, 51.2, 0.00},
	        {"adMCL", femur, new Point3d(398, 1381, 847), tibiaFibula, new Point3d(394, 1349, 848), 1500, 27.2, 0.00, meniscus, new Point3d(395.34588, 1357.005, 847.59604)},
	        {"pdMCL", femur, new Point3d(398, 1382, 838), tibiaFibula, new Point3d(395, 1348, 838), 1500, 23.8, 0.00, meniscus, new Point3d(395.88117, 1357.3174, 836.81185)},
	        {"asMCL", femur, new Point3d(398, 1384, 849), tibiaFibula, new Point3d(375, 1301, 851), 2500, 40.3, 0.00, tibiaFibula, new Point3d(395, 1345, 850)},
	        {"msMCL", femur, new Point3d(399, 1385, 845), tibiaFibula, new Point3d(376, 1297, 848), 2600, 38.6, 0.00, tibiaFibula, new Point3d(395, 1345, 847)},
	        {"psMCL", femur, new Point3d(399, 1384, 840), tibiaFibula, new Point3d(376, 1293, 846), 2700, 37.1, 0.00, tibiaFibula, new Point3d(396, 1345, 844)},
	        {"aPCL",  femur, new Point3d(365, 1377, 849), tibiaFibula, new Point3d(360, 1354, 828), 12500, 39.7, 0.00},
	        {"pPCL",  femur, new Point3d(370, 1374, 842), tibiaFibula, new Point3d(362, 1350, 825), 1500, 38.4, 0.00},
	        {"POL",   femur, new Point3d(399, 1388, 836), tibiaFibula, new Point3d(393, 1343, 828), 1600, 44.8, 0.00},
	        {"lPL",   patella, new Point3d(347, 1382, 895), tibiaFibula, new Point3d(350, 1326, 869), 6000, 55.8, 0.003},
	        {"cPL",   patella, new Point3d(356, 1373, 894), tibiaFibula, new Point3d(359, 1328, 871), 4800, 45.6, 0.003},
	        {"mPL",   patella, new Point3d(364, 1381, 897), tibiaFibula, new Point3d(364, 1331, 870), 3200, 50.0, 0.003}
	    };
		for (Object[] lig : ligaments) {
			String name = (String) lig[0];
			RigidBody body1 = (RigidBody) lig[1];
			Point3d p1 = (Point3d) lig[2];
			RigidBody body2 = (RigidBody) lig[3];
			Point3d p2 = (Point3d) lig[4];
			double stiffness = Double.valueOf(lig[5].toString());
			double refLength = Double.valueOf(lig[6].toString());
			double damping = Double.valueOf(lig[7].toString());
			FrameMarker via0 = new FrameMarker();
			FrameMarker via1 = new FrameMarker();
			mech.addFrameMarker(via0, body1, p1);
			mech.addFrameMarker(via1, body2, p2);
			MultiPointSpring ligament = new MultiPointSpring(name);
			ligament.setMaterial(new Blankevoort1991AxialLigament(stiffness, refLength, damping));
			if (lig.length == 10) {
				RigidBody body3 = (RigidBody) lig[8];
				Point3d p3 = (Point3d) lig[9];
				FrameMarker viaMid = new FrameMarker();
				mech.addFrameMarker(viaMid, body3, p3);
				ligament.addPoint(via0);
				ligament.addPoint(viaMid);
				ligament.addPoint(via1);
				RenderProps.setSphericalPoints(viaMid, 1, Color.GREEN);
			} else {
				ligament.addPoint(via0);
				ligament.addPoint(via1);
			}
			mech.addMultiPointSpring(ligament);
			RenderProps.setSphericalPoints(via0, 1, Color.BLUE);
			RenderProps.setSphericalPoints(via1, 1, Color.BLUE);
			RenderProps.setCylindricalLines(ligament, 0.3, Color.white);
		}
		
		// muscle between Femur and Patella
		Point3d[] patellaPoints = { 
				new Point3d(355.54824, 1407.7581, 899.18102),
				new Point3d(367.64791, 1410.5255, 895.69954),
				new Point3d(343.9282, 1410.4975, 894.20783)};
		String[] muscleNames = { "musM", "musL", "musR" };
		FrameMarker viaFemur = new FrameMarker();
		mech.addFrameMarker(viaFemur, femur, new Point3d(349.42842, 1487.11, 857.06726));
		for (int i = 0; i < patellaPoints.length; i++) {
			FrameMarker viaPatella = new FrameMarker();
			mech.addFrameMarker(viaPatella, patella, patellaPoints[i]);
			MultiPointMuscle muscle = new MultiPointMuscle(muscleNames[i]);
			muscle.addPoint(viaFemur);
			muscle.setSegmentWrappable(10);
			muscle.addPoint(viaPatella);
			muscle.addWrappable(Femur);
			muscle.setRestLength(muscle.getLength());
			muscle.setMaterial(new SimpleAxialMuscle(2, 0.5, 0));
			mech.addMultiPointSpring(muscle);
			RenderProps.setSphericalPoints(viaFemur, 1, Color.BLUE);
			RenderProps.setSphericalPoints(viaPatella, 1, Color.BLUE);
			RenderProps.setCylindricalLines(muscle, 0.6, Color.RED);
		}
	}
	// create joint
	private JointBase createJoint(ConnectableBody femur, ConnectableBody tifi) {
		Vector3d origin = new Vector3d(360, 1380, 840);
		RigidTransform3d TDW = new RigidTransform3d(origin.x, origin.y, origin.z);
		TDW.setRpyDeg(0, 90, 0);
		HingeJoint joint = new HingeJoint(femur, tifi, TDW);
		mech.addBodyConnector(joint);
		setJointCompliance(joint, 1e-4, 1e2);
		setJointRenderProps(joint);
		JointControl(joint);
		return joint;
	}
	// set Compliance for Joint
	private void setJointCompliance(JointBase joint, double complianceValue, double dampingValue) {
		VectorNd comp = new VectorNd(joint.numConstraints());
		VectorNd damp = new VectorNd(joint.numConstraints());
		for (int i = 0; i < joint.numConstraints(); i++) {
			comp.set(i, complianceValue);
			damp.set(i, dampingValue);
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
		panel.addWidget(joint, "theta");
		panel.addWidget(joint, "compliance");
		panel.addWidget(joint, "damping");
		addControlPanel(panel);
	}

	// set collision Behavior
	private void setCollisionBehavior(Collidable body1, Collidable body2, double mu, double compliance,
			double damping) {
		CollisionBehavior behavior = new CollisionBehavior(true, mu);
		behavior.setCompliance(compliance);
		behavior.setDamping(damping);
		mech.setCollisionBehavior(body1, body2, behavior);
	}
	
	// set a contact monitor
	private class ContactMonitor extends MonitorBase {
		public void apply(double t0, double t1) {
			// get the contacts from the collision response and print their
			// positions and forces.
			List<ContactData> cdata = resp.getContactData();
			if (cdata.size() > 0) {
				System.out.println("num contacts: " + cdata.size() + ", time=" + t0);
				double contactSum = 0;
				for (ContactData cd : cdata) {
					System.out.print(" pos:   " + cd.getPosition0().toString("%8.3f"));
					System.out.println(", force: " + cd.getContactForce().toString("%8.1f"));
					contactSum = contactSum + cd.getContactForceScalar();
				}
			}
		}
	}
	
	// set collision manager
	private void setCollisionManager() {
		CollisionManager cm = mech.getCollisionManager();
		RenderProps.setVisible(cm, true);
		cm.setDrawContactForces(false);
		cm.setDrawFrictionForces(false);
		cm.setContactForceLenScale(0.1);
		RenderProps.setSolidArrowLines(cm, 0.2, Color.RED);
		cm.setDrawIntersectionPoints(true);
		RenderProps.setSphericalPoints(cm, 0.5, Color.GREEN);
	}
	
	private void addProbe() throws IOException {
//		FrameMarker mkrProbe = mech.addFrameMarker(Femur, new Point3d(331.77735, 1701.131, 795.7149));
		FrameMarker mkrProbe = mech.addFrameMarker(TibiaFibula, new Point3d(380, 1073, 828));
		mkrProbe.setName("ForceProbe");
		RenderProps.setSphericalPoints(mkrProbe, 4, Color.BLUE);

		// create a InputProbe
		NumericInputProbe ForceProbe = new NumericInputProbe(mkrProbe, "externalForce",
				PathFinder.getSourceRelativePath(this, "ForceforFemur.txt"));
		ForceProbe.setName("force");
		addInputProbe(ForceProbe);

		// create a OutputProbe
		NumericOutputProbe PosProbe = new NumericOutputProbe(mkrProbe, "externalForce", 0, 5, -1);
		PosProbe.setName("position");
		addOutputProbe(PosProbe);

		NumericOutputProbe MAPstressProbe = new NumericOutputProbe(meshFemurCart, "nodes/0:MAPStress", 0, 5, -1);
		MAPstressProbe.setName("MAPstress");
		addOutputProbe(MAPstressProbe);

		NumericOutputProbe displacementProbe = new NumericOutputProbe(meshFemurCart, "nodes/0:displacement", 0, 5, -1);
		displacementProbe.setName("displacement");
		addOutputProbe(displacementProbe);
	}
}