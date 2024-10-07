package lowerlimb;

import java.awt.Color;
import java.io.IOException;
import artisynth.core.femmodels.AnsysCdbReader;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
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
	MechModel mech_femur;
	MechModel mech_tifi;
	
	//create fem
    FemModel3d femur;
    FemModel3d tifi;
    
	@Override
	public void build(String[] args) throws IOException {
		
		//import lowerlimb_mesh_data
		mech_femur = new MechModel("mech_femur");
		mech_tifi = new MechModel("mech_tifi");
		
	    femur = new FemModel3d("femur");
	    tifi = new FemModel3d("tifi");
		
        try {

    		femur = AnsysCdbReader.read(geodata+"Femur.cdb");
    		tifi = AnsysCdbReader.read(geodata+"TibiaFibula.cdb");
    		
        } catch (IOException ioe) {

        	throw new RuntimeException("Failed to read model", ioe);
        }
        
        //Set physical properties
        
        //add models
        mech_femur.addModel(femur);
        mech_tifi.addModel(tifi);
        
        addModel(mech_femur);
        addModel(mech_tifi);
        
        //set render properties
        setRenderProps (femur);
        setRenderProps (tifi);
    
        
//        Rendering and Visualizations
//        RenderProps.setVisible(femur, true);
        
//        Nodes
//        RenderProps.setSphericalPoints(femur, 0.1, Color.RED);
        
//        Elements
//        RenderProps.setLineColor(femur, Color.CYAN);
//        RenderProps.setLineWidth(femur, 2);
        
//        femur.setElementWidgetSize (0.7);
//        RenderProps.setFaceColor (femur, new Color(0.7f, 0.7f, 1f));
//        RenderProps.setLineWidth(femur, 0);

//        Surface and other meshes
//        femur.setSurfaceRendering(SurfaceRender.Shaded);
//        RenderProps.setFaceColor(femur, new Color (0.7f, 0.7f, 1f));
//        RenderProps.setDrawEdges (femur, true);
//        RenderProps.setEdgeColor (femur, Color.BLUE);
        
//        femur.setSurfaceRendering(SurfaceRender.Stress);
//        femur.setStressPlotRanging(Ranging.Auto);
//        RenderProps.setDrawEdges (femur, true);
//        RenderProps.setEdgeColor (femur, Color.BLUE);
        
//        Create a colorbar
//        ColorBar cbar = new ColorBar();
//        cbar.setName("colorBar");
//        cbar.setNumberFormat("%.2f");      // 2 decimal places
//        cbar.populateLabels(0.0, 1.0, 10); // Start with range [0,1], 10 ticks
//        cbar.setLocation(-100, 0.1, 20, 0.8);
//        addRenderable(cbar);      
	}
	
	protected void setRenderProps (FemModel3d fem) {
		fem.setSurfaceRendering(SurfaceRender.Shaded);
		RenderProps.setLineColor (fem, Color.BLUE);
		RenderProps.setFaceColor(fem, new Color (0.5f, 0.5f, 1f));
	}
	
//	@Override
//	public void prerender(RenderList list) {
//		super.prerender(list);
//		// Synchronize color bar/values in case they are changed. Do this *after*
//		// super.prerender(), in case values are changed there.
//		ColorBar cbar = (ColorBar)(renderables().get("colorBar"));
//		cbar.setColorMap(femur.getColorMap());
//		DoubleInterval range = femur.getStressPlotRange();
//		cbar.updateLabels(range.getLowerBound(), range.getUpperBound());
//	}
}
