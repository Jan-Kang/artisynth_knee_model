package artisynth_knee_model;

import java.awt.Color;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.List;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.fields.ScalarNodalField;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.materials.Blankevoort1991AxialLigament;
import artisynth.core.materials.FemMaterial;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.SimpleAxialMuscle;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.ConnectableBody;
import artisynth.core.mechmodels.ContactData;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.HingeJoint;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.MultiPointSpring;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.probes.DataFunction;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericMonitorProbe;
import artisynth.core.probes.NumericOutputProbe;
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
import maspack.render.Renderer.PointStyle;
import maspack.util.Clonable;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

public class knee_model extends RootModel {

	// Get the data path
	String myPath = maspack.util.PathFinder.getSourceRelativePath(this, "data/");
	// Create a new mechanical model
	MechModel myMech = new MechModel();
	// Value for rigid body
	RigidBody Femur, Meniscus, TibiaFibula, TibiaCart, Patella, PatellaCart;
	// Value for fem model
	FemModel3d meshFemurCart;
	// Create a joint value between Femur and TibialFibula
	JointBase myJoint;
	// Create a collision responses
	CollisionResponse myResp;

	public void build(String[] args) throws IOException {
		
		addModel(myMech);

		// Set a gravity
		// Units: m/s^2
		myMech.setGravity(0, -9810, 0);

		// Import rigid bodies
		// All model dimensions are originally in mm.
		// To ensure unit consistency, all dimensions are converted to meters.
		// Bone, density in kg/m³
		Femur = importRigidBody("femur.obj", "Femur", 1.3e-6);
		TibiaFibula = importRigidBody("tibia.obj", "TibiaFibula", 1.3e-6);
		Patella = importRigidBody("patella.obj", "Patella", 1.3e-6);
		// Cartilage, density in kg/m³
		TibiaCart = importRigidBody("tibcart.obj", "TibiaCart", 1.15e-6);
		PatellaCart = importRigidBody("patcart.obj", "PatellaCart", 1.15e-6);
		Meniscus = importRigidBody("Meniscus.obj", "Meniscus", 1.1e-6);

		// Import fem models
		meshFemurCart = importFemModel("mesh_FemurCart.cdb", "FemurCart", 1.15e-6, // density in kg/m³
				0.02, // mass damping
				50, // stiffness damping
				new LinearMaterial(/* Young's modulu(MPa): */10, 
						           /* Poisson's ratio: */    0.45));

		// Import ligaments
		addLigaments(Femur, TibiaFibula, Meniscus, Patella);

		// Connecting between rigid body models
		myMech.attachFrame(Meniscus, TibiaCart);
		myMech.attachFrame(TibiaCart, TibiaFibula);
		myMech.attachFrame(PatellaCart, Patella);

		// Connecting the fem model to rigid body model with nodes of mesh model
		PolygonalMesh femurSurface = Femur.getSurfaceMesh();
		double tol = 1;
		for (FemNode3d n : meshFemurCart.getNodes()) {
			if (meshFemurCart.isSurfaceNode(n)) {
				Point3d n_Position = n.getPosition();
				if (femurSurface.distanceToPoint(n_Position) < tol) {
//					RenderProps.setVisible(n, true);
//					RenderProps.setSphericalPoints(n, 0.1, Color.red);
					myMech.attachPoint(n, Femur);
				}
			}
		}

		// Set a joint between femur und tibiafibula
		myJoint = createJoint(Femur, TibiaFibula);
		
		// Set dynamic property of models
		Femur.setDynamic(false);
		TibiaFibula.setDynamic(true);

		// Set collision behaviors of models
		// Body1, Body2, mu, compliance, damping
		setCollisionBehavior(Femur, PatellaCart, 0, 1e-6, 1e6);
		setCollisionBehavior(meshFemurCart, PatellaCart, 0, 1e-6, 1e6);
		setCollisionBehavior(meshFemurCart, Meniscus, 0, 1e-6, 1e6);
		setCollisionBehavior(meshFemurCart, TibiaCart, 0, 1e-6, 1e6);

		// Enable collision force visualization
		setCollisionManager();

		// Set a collision response
		myResp = myMech.setCollisionResponse(meshFemurCart, Meniscus);
		addMonitor (new ContactMonitor());
		
		// Add a color bar
		createColorBar();

		// Initialize writerfile to record Displacement, Stress, Strain of nodes
		initialWriter();

		// Add a probe to monitor or collect specific data during the simulation
		addProbe();

		// Set a stop time of the simulation
		addBreakPoint(4.0);
	}

	private RigidBody importRigidBody(
			String filename, 
			String modelname, 
			double density) throws IOException {
		PolygonalMesh mesh = new PolygonalMesh(myPath + filename);
		RigidBody body = RigidBody.createFromMesh(modelname, mesh, density, 1);
		myMech.addRigidBody(body);
		return body;
	}

	private FemModel3d importFemModel(
			String filename, 
			String modelname, 
			double density, 
			double massDamping,
			double stiffnessDamping, 
			FemMaterial material) throws IOException {
		FemModel3d femModel = AnsysCdbReader.read(myPath + filename);
		femModel.setName(modelname);
		femModel.setDensity(density);
		femModel.setMassDamping(massDamping);
		femModel.setStiffnessDamping(stiffnessDamping);
		femModel.setMaterial(material);
		myMech.addModel(femModel);
		setFemRenderProps(femModel);
		femModel.setComputeNodalStress(true);
		femModel.setComputeNodalStrain(true);
		return femModel;
	}

	private void setFemRenderProps(FemModel3d fem) {
//		fem.setSurfaceRendering(SurfaceRender.Shaded);
		fem.setSurfaceRendering(SurfaceRender.Stress);
		fem.setStressPlotRanging(Ranging.Auto);
//		RenderProps.setVisible(fem.getNodes(), false);
//		RenderProps.setVisible(fem.getElements(), false);
//		RenderProps.setAlpha(fem, 1.0);
//		RenderProps.setFaceColor (fem, Color.GRAY);
//		RenderProps.setLineColor(fem, Color.DARK_GRAY);
//		RenderProps.setSphericalPoints (fem, 0.2, Color.CYAN);
	}

	private double minValue;
	private double maxValue;

	public ColorBar createColorBar() {
		ColorBar cbar = new ColorBar();
		cbar.setName("colorBar");
		cbar.setNumberFormat("%.2f");
		cbar.populateLabels(0.0, 0.1, 10);
		cbar.setLocation(-100, 0.1, 20, 0.8);
		addRenderable(cbar);
		return cbar;
	}

	public void prerender(RenderList list) {
		// Add a Scalar field
//		addScalarField();
		// Update cbar values
		super.prerender(list);
		ColorBar cbar = (ColorBar) (renderables().get("colorBar"));
		List<FemModel3d> femModels = Arrays.asList(meshFemurCart);
		for (FemModel3d fem : femModels) {
			cbar.setColorMap(fem.getColorMap());
			DoubleInterval range = fem.getStressPlotRange();
			cbar.updateLabels(range.getLowerBound(), range.getUpperBound());
//			cbar.updateLabels(minValue, maxValue);
		}
	}

	private void addScalarField() {
		ScalarNodalField field = new ScalarNodalField(meshFemurCart);
		meshFemurCart.addField(field);
		for (FemNode3d n : meshFemurCart.getNodes()) {
			if (meshFemurCart.isSurfaceNode(n)) {
				double value = n.getMAPStress();
				if (value < minValue) {
					minValue = value;
				}
				if (value > maxValue) {
					maxValue = value;
				}
				field.setValue(n, value);
			}
		}
		field.setVisualization(ScalarNodalField.Visualization.POINT);
		RenderProps.setPointRadius(field, 0.5);
		RenderProps.setPointStyle(field, PointStyle.SPHERE);
	}

	private void addLigaments(RigidBody femur, RigidBody tibiaFibula, RigidBody meniscus, RigidBody patella) {
		// Data of ligaments
		Object[][] ligaments = {
		    {"ALL",   femur, new Point3d(325, 1377, 842), tibiaFibula, new Point3d(324, 1352, 844), 3000, 15, 0.00, meniscus, new Point3d(324, 1362, 843)},	        
		    {"aACL",  femur, new Point3d(356, 1384, 834), tibiaFibula, new Point3d(362, 1357, 853), 6200, 32.3, 0.00},
	        {"pACL",  femur, new Point3d(356, 1379, 832), tibiaFibula, new Point3d(358, 1357, 850), 3400, 26.6, 0.00},
	        {"aLCL",  femur, new Point3d(326, 1381, 841), tibiaFibula, new Point3d(311, 1328, 829), 8000, 45, 0.00},
	        {"mLCL",  femur, new Point3d(327, 1381, 837), tibiaFibula, new Point3d(311, 1328, 825), 8000, 40.2, 0.00},
	        {"pLCL",  femur, new Point3d(329, 1380, 833), tibiaFibula, new Point3d(313, 1331, 823), 8000, 40.2, 0.00},
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
		// Read ligaments data
		for (Object[] lig : ligaments) {
			String name = (String) lig[0];
			RigidBody body1 = (RigidBody) lig[1];
			Point3d p1 = (Point3d) lig[2];
			RigidBody body2 = (RigidBody) lig[3];
			Point3d p2 = (Point3d) lig[4];
			double stiffness = Double.valueOf(lig[5].toString());
			double refLength = Double.valueOf(lig[6].toString());
			double damping = Double.valueOf(lig[7].toString());
			// Create frame marker for connecting between ligaments
			FrameMarker via0 = new FrameMarker();
			FrameMarker via1 = new FrameMarker();
			myMech.addFrameMarker(via0, body1, p1);
			myMech.addFrameMarker(via1, body2, p2);
			// Create multi-point spring
			MultiPointSpring ligament = new MultiPointSpring(name);
			ligament.setMaterial(new Blankevoort1991AxialLigament(stiffness, refLength, damping));
			if (lig.length == 10) {
				RigidBody body3 = (RigidBody) lig[8];
				Point3d p3 = (Point3d) lig[9];
				FrameMarker viaMid = new FrameMarker();
				myMech.addFrameMarker(viaMid, body3, p3);
				ligament.addPoint(via0);
				ligament.addPoint(viaMid);
				ligament.addPoint(via1);
				RenderProps.setSphericalPoints(viaMid, 1, Color.GREEN);
			} else {
				ligament.addPoint(via0);
				ligament.addPoint(via1);
			}
			myMech.addMultiPointSpring(ligament);
			// Set render properties for spring
			RenderProps.setSphericalPoints(via0, 1, Color.BLUE);
			RenderProps.setSphericalPoints(via1, 1, Color.BLUE);
			RenderProps.setCylindricalLines(ligament, 0.3, Color.white);
		}
		// Create muscle between Femur and Patella
		Point3d[] patellaPoints = { 
				new Point3d(355.54824, 1407.7581, 899.18102),
				new Point3d(367.64791, 1410.5255, 895.69954),
				new Point3d(343.9282, 1410.4975, 894.20783)};
		String[] muscleNames = { "musM", "musL", "musR" };
		FrameMarker viaFemur = new FrameMarker();
		myMech.addFrameMarker(viaFemur, femur, new Point3d(349.42842, 1487.11, 857.06726));
		for (int i = 0; i < patellaPoints.length; i++) {
			FrameMarker viaPatella = new FrameMarker();
			myMech.addFrameMarker(viaPatella, patella, patellaPoints[i]);
			MultiPointMuscle muscle = new MultiPointMuscle(muscleNames[i]);
			muscle.addPoint(viaFemur);
			muscle.setSegmentWrappable(10);
			muscle.addPoint(viaPatella);
			muscle.addWrappable(Femur);
			muscle.setRestLength(muscle.getLength());
			muscle.setMaterial(new SimpleAxialMuscle(2, 0.5, 0));
			myMech.addMultiPointSpring(muscle);
			RenderProps.setSphericalPoints(viaFemur, 1, Color.BLUE);
			RenderProps.setSphericalPoints(viaPatella, 1, Color.BLUE);
			RenderProps.setCylindricalLines(muscle, 0.6, Color.RED);
		}
	}

	private JointBase createJoint(ConnectableBody femur, ConnectableBody tifi) {
		Vector3d origin = new Vector3d(360, 1380, 840);
		RigidTransform3d TDW = new RigidTransform3d(origin.x, origin.y, origin.z);
		TDW.setRpyDeg(-5, 90, 0);
		HingeJoint joint = new HingeJoint(femur, tifi, TDW);
		myMech.addBodyConnector(joint);
		setJointCompliance(joint, 1e-4, 1e2);
		setJointRenderProps(joint);
		JointControl(joint);
		return joint;
	}

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

	private void setJointRenderProps(JointBase joint) {
		joint.setShaftLength(70);
		joint.setShaftRadius(1);
		joint.setAxisLength(50);
		joint.setDrawFrameC(AxisDrawStyle.ARROW);
		joint.setDrawFrameD(AxisDrawStyle.ARROW);
	}

	private void JointControl(JointBase joint) {
		ControlPanel panel = new ControlPanel();
		panel.addWidget(joint, "theta");
		panel.addWidget(joint, "compliance");
		panel.addWidget(joint, "damping");
		addControlPanel(panel);
	}

	private void setCollisionBehavior(
			Collidable body1, 
			Collidable body2, 
			double mu, 
			double compliance,
			double damping) {
		CollisionBehavior behavior = new CollisionBehavior(true, mu);
		behavior.setCompliance(compliance);
		behavior.setDamping(damping);
		myMech.setCollisionBehavior(body1, body2, behavior);
	}
	
	private void setCollisionManager() {
		CollisionManager cm = myMech.getCollisionManager();
		RenderProps.setVisible(cm, true);
		cm.setDrawContactForces(false);
		cm.setDrawFrictionForces(false);
		cm.setContactForceLenScale(0.1);
		RenderProps.setSolidArrowLines(cm, 0.2, Color.RED);
		cm.setDrawIntersectionPoints(true);
		RenderProps.setSphericalPoints(cm, 0.5, Color.GREEN);
	}
	
	private class ContactMonitor extends MonitorBase {
		public void apply(double t0, double t1) {
			// get the contacts from the collision response and print their
			// positions and forces.
			List<ContactData> cdata = myResp.getContactData();
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
	
	private void addProbe() throws IOException {
		FrameMarker mkrProbe = myMech.addFrameMarker(TibiaFibula, new Point3d(380, 1073, 828));
		mkrProbe.setName("ForceProbe");
		RenderProps.setSphericalPoints(mkrProbe, 4, Color.BLUE);

		// create a InputProbe
		NumericInputProbe ForceProbe = new NumericInputProbe(mkrProbe, "externalForce",
				PathFinder.getSourceRelativePath(this, "ForceforFemur.txt"));
		ForceProbe.setName("OutputProbe_force");
		addInputProbe(ForceProbe);

		// create a OutputProbe
		NumericOutputProbe PosProbe = new NumericOutputProbe(mkrProbe, "displacement", 
				0, 20, -1);
		PosProbe.setName("OutputProbe_displacement");
		addOutputProbe(PosProbe);
		
		// Create a MonitorProbe
		NumericMonitorProbe DispProbe = new NumericMonitorProbe(meshFemurCart.numNodes() * 3,
				myPath + "output/Displacement.dat", 0, 20, -1);
		DispProbe.setName("Displacement");
		DispProbe.setDataFunction(new FEMDisplacementFunction());
		addOutputProbe(DispProbe);

		NumericMonitorProbe StressProbe = new NumericMonitorProbe(meshFemurCart.numNodes() * 9,
				myPath + "output/Stress.dat", 0, 20, -1);
		StressProbe.setName("Stress");
		StressProbe.setDataFunction(new FEMStressFunction());
		addOutputProbe(StressProbe);

		NumericMonitorProbe StrainProbe = new NumericMonitorProbe(meshFemurCart.numNodes() * 9,
				myPath + "output/Strain.dat", 0, 20, -1);
		StrainProbe.setName("Strain");
		StrainProbe.setDataFunction(new FEMStrainFunction());
		addOutputProbe(StrainProbe);
	}
	
	public PrintWriter writerPosition;
	public PrintWriter writerStress;
	public PrintWriter writerStrain;

	public void initialWriter() throws IOException {
		try {
			writerPosition = new PrintWriter(new FileWriter(myPath + "output/Displacement.txt", false));
			writerStress = new PrintWriter(new FileWriter(myPath + "output/Stress.txt", false));
			writerStrain = new PrintWriter(new FileWriter(myPath + "output/Strain.txt", false));
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public class FEMDisplacementFunction implements DataFunction, Clonable {
		public void eval(VectorNd vec, double t, double trel) {
			int idx = 0;
			writerPosition.println("Time:" + t);
			for (FemNode3d n : meshFemurCart.getNodes()) {
				if (meshFemurCart.isSurfaceNode(n)) {
					vec.set(idx++, n.getDisplacement().x);
					vec.set(idx++, n.getDisplacement().y);
					vec.set(idx++, n.getDisplacement().z);
					writerPosition.print("Node Id:" + n.getNumber());
					writerPosition.print(" x: " + n.getDisplacement().x);
					writerPosition.print(" y: " + n.getDisplacement().y);
					writerPosition.print(" z: " + n.getDisplacement().z);
					writerPosition.println();
				}
			}
		}

		public Object clone() throws CloneNotSupportedException {
			return super.clone();
		}
	}

	public class FEMStressFunction implements DataFunction, Clonable {
		public void eval(VectorNd vec, double t, double trel) {
			int idx = 0;
			writerStress.println("Time:" + t);
			for (FemNode3d n : meshFemurCart.getNodes()) {
				if (meshFemurCart.isSurfaceNode(n)) {
					vec.set(idx++, n.getStress().m00);
					vec.set(idx++, n.getStress().m01);
					vec.set(idx++, n.getStress().m02);
					vec.set(idx++, n.getStress().m10);
					vec.set(idx++, n.getStress().m11);
					vec.set(idx++, n.getStress().m12);
					vec.set(idx++, n.getStress().m20);
					vec.set(idx++, n.getStress().m21);
					vec.set(idx++, n.getStress().m22);
					writerStress.print("Node Id:" + n.getNumber());
					writerStress.print(" stress: " + n.getStress());
				}
			}
		}

		public Object clone() throws CloneNotSupportedException {
			return super.clone();
		}
	}

	public class FEMStrainFunction implements DataFunction, Clonable {
		public void eval(VectorNd vec, double t, double trel) {
			int idx = 0;
			writerStrain.println("Time:" + t);
			for (FemNode3d n : meshFemurCart.getNodes()) {
				if (meshFemurCart.isSurfaceNode(n)) {
					vec.set(idx++, n.getStrain().m00);
					vec.set(idx++, n.getStrain().m01);
					vec.set(idx++, n.getStrain().m02);
					vec.set(idx++, n.getStrain().m10);
					vec.set(idx++, n.getStrain().m11);
					vec.set(idx++, n.getStrain().m12);
					vec.set(idx++, n.getStrain().m20);
					vec.set(idx++, n.getStrain().m21);
					vec.set(idx++, n.getStrain().m22);
					writerStrain.print("Node Id:" + n.getNumber());
					writerStrain.print(" Strain: " + n.getStrain());
				}
			}
		}

		public Object clone() throws CloneNotSupportedException {
			return super.clone();
		}
	}
}