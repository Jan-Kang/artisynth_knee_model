package lowerlimb;

import java.io.IOException;

import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.workspace.RootModel;
import maspack.util.PathFinder;

public class lowerlimb extends RootModel {
	
	String geodata = PathFinder.getSourceRelativePath(this, "data/");
	
	//import of femur_mesh
	public void build(String[] args) throws IOException {
        
		MechModel mech = new MechModel("mech");
        FemModel3d femur = AnsysCdbReader.read(geodata+"Femur.cdb");
        
        mech.addModel(femur);
        addModel(mech);
	}
}
