package lowerlimb;

import java.io.IOException;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import maspack.util.PathFinder;

public class lowerlimb extends RootModel {
	
	String geodata = PathFinder.getSourceRelativePath(this, "data/");
	

	public void build(String[] args) throws IOException {
        
		//import of femur_mesh
		MechModel mech_femur = new MechModel("mech_femur");
        FemModel3d femur = AnsysCdbReader.read(geodata+"Femur.cdb");
        
        mech_femur.addModel(femur);
        addModel(mech_femur);
        
		//import of TibiaFibula_mesh
		MechModel mech_tifi = new MechModel("mech_tifi");
        FemModel3d tifi = AnsysCdbReader.read(geodata+"TibiaFibula.cdb");
        
        mech_tifi.addModel(tifi);
        addModel(mech_tifi);
	}
}
