package Knee_model;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.workspace.RootModel;
import maspack.geometry.PolygonalMesh;
import maspack.render.RenderProps;

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
	
	@Override
	public void build (String [] args) throws IOException {
		// set a gravity
		mech.setGravity (0, 0, -9.81);
		// create rigid model
	    FemurRigid = importFemurRigid ();
	    // TibiaFibulaRigid = importTibiaFibulaRigid ();
	    // create FEM model
		Femur = importFemur ();
		// FemurCart = importFemurCart ();
		// Mensicus = importMensicus ();
		// TibiaCart = importTibiaCart ();
		// TibiaFibula = importTibiaFibula ();
		// Patella = importPatella ();
		// PatellaCart = importPatellaCart ();
		addModel (mech);
	}
	// import Rigid model
	private RigidBody importFemurRigid () throws IOException {
		PolygonalMesh meshFemur = null;
		meshFemur = new PolygonalMesh (Modeldata + "mesh_femur_rigid.obj");
		RigidBody FemurRigid = 
				RigidBody.createFromMesh("FemurRigid", meshFemur, 10, 1);
		mech.addRigidBody(FemurRigid);
		return FemurRigid;
	}	
	private RigidBody importTibiaFibulaRigid () throws IOException {
		PolygonalMesh meshTibiaFibulaRigid = null;
		meshTibiaFibulaRigid = new PolygonalMesh (Modeldata + "mesh_TibiaFibula_rigid.obj");
		RigidBody TibiaFibulaRigid = 
				RigidBody.createFromMesh("TibiaFibulaRigid", meshTibiaFibulaRigid, 10, 1);
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
	private FemModel3d importFemurCart () throws IOException {
		// import model
		FemModel3d FemurCart = null;
		FemurCart = new FemModel3d ("FemurCart");
		FemurCart = AnsysCdbReader.read (Modeldata + "mesh_FemurCart.cdb");
		// set physical properties
		FemurCart.setDensity (1.9e-6);
		FemurCart.setMassDamping (0.01);
		FemurCart.setStiffnessDamping (0.02);
		FemurCart.setMaterial (new LinearMaterial (1e9, 0.3));
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
		Mensicus.setDensity (1.9e-6);
		Mensicus.setMassDamping (0.01);
		Mensicus.setStiffnessDamping (0.02);
		Mensicus.setMaterial (new LinearMaterial (1e9, 0.3));
		Mensicus.setName ("Mensicus");
		if (Mensicus.isVolumeValid ())
			System.out.println ("Mensicus mesh valid.");
		mech.addModel (Mensicus);
		setFemRenderProps (Mensicus);
		return FemurCart;
	}
	private FemModel3d importTibiaCart () throws IOException {
		// import model
		FemModel3d TibiaCart = null;
		TibiaCart = new FemModel3d ("TibiaCart");
		TibiaCart = AnsysCdbReader.read (Modeldata + "mesh_TibiaCart.cdb");
		// set physical properties
		TibiaCart.setDensity (1.9e-6);
		TibiaCart.setMassDamping (0.01);
		TibiaCart.setStiffnessDamping (0.02);
		TibiaCart.setMaterial (new LinearMaterial (1e9, 0.3));
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
		TibiaFibula.setDensity (1.9e-6);
		TibiaFibula.setMassDamping (0.01);
		TibiaFibula.setStiffnessDamping (0.02);
		TibiaFibula.setMaterial (new LinearMaterial (1e9, 0.3));
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
		Patella.setDensity (1.9e-6);
		Patella.setMassDamping (0.01);
		Patella.setStiffnessDamping (0.02);
		Patella.setMaterial (new LinearMaterial (1e9, 0.3));
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
		PatellaCart.setDensity (1.9e-6);
		PatellaCart.setMassDamping (0.01);
		PatellaCart.setStiffnessDamping (0.02);
		PatellaCart.setMaterial (new LinearMaterial (1e9, 0.3));
		PatellaCart.setName ("PatellaCart");
		if (PatellaCart.isVolumeValid ())
			System.out.println ("PatellaCart mesh valid.");
		mech.addModel (PatellaCart);
		setFemRenderProps (PatellaCart);
		return PatellaCart;
	}
	// set FEM model render properties
	private void setFemRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering (SurfaceRender.Shaded);
		RenderProps.setAlpha(fem, 1.0);
		RenderProps.setLineColor (fem, Color.darkGray);
		RenderProps.setFaceColor (fem, Color.LIGHT_GRAY);
		// RenderProps.setSphericalPoints (fem, 0.5, Color.CYAN);
	}
}