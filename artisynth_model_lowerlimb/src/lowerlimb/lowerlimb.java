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
import maspack.matrix.Point3d;
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
    FemModel3d cartilage;
    
	@Override
	public void build(String[] args) throws IOException {
		
        //close of Gravity
        mech.setGravity(0, 0, 0);
		
		//import femur
	    femur = new FemModel3d("femur");
		femur = AnsysCdbReader.read(geodata+"Femur.cdb");
        //Set physical properties
        femur.setDensity(1900);
        femur.setMassDamping(0.01);
        femur.setStiffnessDamping(0.02);
        femur.setMaterial(new LinearMaterial(1e9, 0.3));
        // 调用计算股骨尺寸的函数
        computeBoundingBox(femur);
		
		//import tibi
	    tifi = new FemModel3d("tifi");
		tifi = AnsysCdbReader.read(geodata+"TibiaFibula.cdb");
        //Set physical properties
        tifi.setDensity(1900);
        tifi.setMassDamping(0.01);
        tifi.setStiffnessDamping(0.02);
        tifi.setMaterial(new LinearMaterial(1e9, 0.3));
        // 调用计算胫骨尺寸的函数
        computeBoundingBox(tifi);
        
        //create cartilage
        cartilage = new FemModel3d("cartilage");
        // 使用一个简单的几何体创建软骨
        double radius = 100;   // 圆盘半径
        double thickness = 1; // 圆盘厚度

        // 3. 创建圆盘（实际上是一个薄的圆柱体）
        FemFactory.createCylinder(cartilage, thickness, radius, 20, 10, 20); // 创建一个圆柱，20为圆的细分数


        // 3. 设置软骨的物理属性
        cartilage.setDensity(1000);  // 软骨的密度
        cartilage.setMaterial(new LinearMaterial(1e7, 0.4));  // 软骨的材料属性

        // 4. 设置软骨位置：在股骨下方
        Point3d cartilagePosition = new Point3d(
            (36.6293 + 122.6768) / 2,  // X 方向取中点
            (-68.1622 + 51.3401702) / 2,// Y 方向取股骨的 Y 最小值
            -435.0917            // Z 方向放置在股骨下方，向下偏移 20 单位（软骨厚度）
        );
        
        for (FemNode3d node : cartilage.getNodes()) {
            Point3d currentPosition = node.getPosition();
            // 更新节点位置
            node.setPosition(new Point3d(
                currentPosition.x + cartilagePosition.x,
                currentPosition.y + cartilagePosition.y,
                currentPosition.z + cartilagePosition.z
            ));
        }


        // 5. 添加软骨模型到机电模型
        mech.addModel(cartilage);

        // 6. 设置渲染属性以便可视化
        RenderProps.setFaceColor(cartilage, new Color(0.5f, 0.8f, 1f));  // 软骨的颜色为淡蓝色
        RenderProps.setLineColor(cartilage, Color.BLUE);  // 软骨边缘颜色
        
        //add models
        mech.addModel(femur);
        mech.addModel(tifi);
        mech.addModel(cartilage);
        
        //open collisionBehavior
        mech.setCollisionBehavior(femur, tifi, true);
        
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
	
	// 计算FEM模型的边界框
	public void computeBoundingBox(FemModel3d fem) {
	    // 初始化最小和最大值
	    Point3d min = new Point3d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
	    Point3d max = new Point3d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

	    // 遍历模型中的所有节点，更新最小和最大坐标值
	    for (FemNode3d node : fem.getNodes()) {
	        Point3d pos = node.getPosition();

	        // 更新最小坐标值
	        if (pos.x < min.x) {
	            min.x = pos.x;
	        }
	        if (pos.y < min.y) {
	            min.y = pos.y;
	        }
	        if (pos.z < min.z) {
	            min.z = pos.z;
	        }

	        // 更新最大坐标值
	        if (pos.x > max.x) {
	            max.x = pos.x;
	        }
	        if (pos.y > max.y) {
	            max.y = pos.y;
	        }
	        if (pos.z > max.z) {
	            max.z = pos.z;
	        }
	    }

	    // 打印模型的大小
	    System.out.println("Model Bounding Box:");
	    System.out.println("Min: " + min.toString());
	    System.out.println("Max: " + max.toString());
	    
	    // 计算并打印模型的尺寸（长宽高）
	    double width = max.x - min.x;
	    double height = max.y - min.y;
	    double depth = max.z - min.z;
	    
	    System.out.println("Model Dimensions (Width, Height, Depth): " + width + ", " + height + ", " + depth);
	}
	
}
