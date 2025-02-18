package testModel;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.ConnectableBody;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.GimbalJoint;
import artisynth.core.mechmodels.HingeJoint;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Particle;
import artisynth.core.mechmodels.PointPlaneForce;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer.AxisDrawStyle;
import maspack.util.PathFinder;

public class testModel extends RootModel {

	MechModel mech = new MechModel();
	RigidBody Femur, FemCart, TiFi, TiFiCart;

	public void build(String[] args) throws IOException {

		mech.setGravity(0, 0, 9.81);

		Femur = RigidBody.createEllipsoid("Femur", 15, 15, 50, 1e-5, 100);
		TiFi = RigidBody.createEllipsoid("TibiaFibula", 15, 15, 50, 1, 100);
		Femur.setPose(new RigidTransform3d(0, 0, 60));
		TiFi.setPose(new RigidTransform3d(0, 0, -60));
		TiFi.setDynamic(false);
		Femur.setDynamic(true);

		// create the PointPlaneForce for the left plane

		mech.addRigidBody(Femur);
		mech.addRigidBody(TiFi);

		createJoint(Femur, TiFi);

		mech.setCollisionBehavior(Femur, TiFi, true, 0.1);
		FrameMarker forcePoint = mech.addFrameMarkerWorld(Femur, new Point3d(0.013430901, -13.349979, 82.741327));
		RenderProps.setSphericalPoints(forcePoint, 3, Color.RED);
		createForceInputProbe();
		addModel(mech);

		RenderProps.setFaceColor(Femur, Color.LIGHT_GRAY);
		RenderProps.setFaceColor(TiFi, Color.LIGHT_GRAY);
		RenderProps.setLineColor(Femur, Color.DARK_GRAY);
		RenderProps.setLineColor(TiFi, Color.DARK_GRAY);

	}

	private void createForceInputProbe() throws IOException {

		NumericInputProbe forceProbe = new NumericInputProbe(mech, "frameMarkers/0:externalForce",
				PathFinder.getSourceRelativePath(this, "ForceforFemur2.txt"));
		forceProbe.setName("External Force");
		addInputProbe(forceProbe);
	}

	// create joint
	private JointBase createJoint(RigidBody femur, RigidBody tifi) {
		Vector3d origin = new Vector3d(0, 0, 0);
		RigidTransform3d TDW = new RigidTransform3d(origin.x, origin.y, origin.z);
		TDW.setRpyDeg(0, 90, 0);
		HingeJoint joint = new HingeJoint(femur, tifi, new Point3d(0, 0, 0), new Vector3d(0, 1, 0));
		// set joint ranges (in degrees)

		// set joint initial value
//		joint.setRoll(0);
//		joint.setPitch(0);
//		joint.setYaw(0);
		// lock a Joint

		mech.addBodyConnector(joint);
		// setJointCompliance(joint);
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
