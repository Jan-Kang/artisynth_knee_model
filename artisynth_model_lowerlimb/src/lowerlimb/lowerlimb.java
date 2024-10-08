package lowerlimb;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.NodeNumberReader;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.renderables.ColorBar;
import artisynth.core.workspace.RootModel;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

public class lowerlimb extends RootModel {
	
	//Path of data
	String geodata = PathFinder.getSourceRelativePath(this, "data/");
	
	//create mech
	MechModel mech = new MechModel();

	//create fem
    FemModel3d femur;
    FemModel3d tifi;
    
	@Override
	public void build(String[] args) throws IOException {
		
		//import lowerlimb_mesh_data
	    femur = new FemModel3d("femur");
	    tifi = new FemModel3d("tifi");
		
        try {

    		femur = AnsysCdbReader.read(geodata+"Femur.cdb");
    		tifi = AnsysCdbReader.read(geodata+"TibiaFibula.cdb");
    		
        } catch (IOException ioe) {

        	throw new RuntimeException("Failed to read model", ioe);
        }
        
        //Set physical properties
        femur.setDensity(1900);
        femur.setMassDamping(0.01);
        femur.setStiffnessDamping(0.02);
        femur.setMaterial(new LinearMaterial(1e9, 0.3));
        
        //Set physical properties
        tifi.setDensity(1900);
        tifi.setMassDamping(0.01);
        tifi.setStiffnessDamping(0.02);
        tifi.setMaterial(new LinearMaterial(1e9, 0.3));
        
        //Fix of Node
        try {
        	ArrayList<FemNode3d> femur_Nodes = NodeNumberReader.read(geodata+"FixNode.txt", femur);
        	for (FemNode3d FixNode : femur_Nodes) {
        		if (FixNode!=null) {
        			FixNode.setDynamic(false);
        		}
        	}
        }
		catch(IOException e) {
			System.out.println("Couldn`t read node file" + geodata + "FixNode.txt");
		}
        
        //add models
        mech.addModel(femur);
        mech.addModel(tifi);
        
        mech.setCollisionBehavior(femur,tifi,true);
        
        addModel(mech);
        
        //set render properties
        setRenderProps (femur);
        setRenderProps (tifi);
           
	}
	
	protected void setRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering(SurfaceRender.Shaded);
		RenderProps.setLineColor (fem, Color.BLUE);
		RenderProps.setFaceColor(fem, new Color (0.5f, 0.5f, 1f));
	}
	
}
