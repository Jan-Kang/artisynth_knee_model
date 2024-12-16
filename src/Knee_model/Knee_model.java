package Knee_model;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import maspack.render.RenderProps;

public class Knee_model extends RootModel {
	// path of data
	String Modeldata = maspack.util.PathFinder.getSourceRelativePath(this, "data/");
	// create MECH
	MechModel mech = new MechModel ();
	// create FEM
	FemModel3d Femur, FemurCart, Mensicus, TibiaCart, TibiaFibula;
	
	@Override
	public void build (String [] args) throws IOException {
		// set a gravity
		mech.setGravity (0, 0, -9.81);
		// create model
		Femur = importFemur ();
		FemurCart = importFemurCart ();
		Mensicus = importMensicus ();
		TibiaCart = importTibiaCart ();
		TibiaFibula = importTibiaFibula ();
	}
	// import FEM model
	private FemModel3d importFemur () throws IOException {
		// import model
		FemModel3d Femur = null;
		Femur = new FemModel3d ("Femur");
		Femur = AnsysCdbReader.read (Modeldata + "mesh_Femur.cdb");
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
		TibiaFibula = AnsysCdbReader.read (Modeldata + "mesh_TibiaFibula.cdb");
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
	// set FEM model render properties
	private void setFemRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering (SurfaceRender.Shaded);
		RenderProps.setAlpha(fem, 1.0);
		RenderProps.setLineColor (fem, Color.darkGray);
		RenderProps.setFaceColor (fem, Color.LIGHT_GRAY);
		// RenderProps.setSphericalPoints (fem, 0.5, Color.CYAN);

	}
}