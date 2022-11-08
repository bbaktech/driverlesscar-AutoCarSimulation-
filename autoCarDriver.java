//autoCarDriver.java

package org.fog.test;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;

import org.cloudbus.cloudsim.Log;
import org.cloudbus.cloudsim.core.CloudSim;
import org.fog.application.AppEdge;
import org.fog.application.AppLoop;
import org.fog.application.Application;
import org.fog.application.selectivity.FractionalSelectivity;
import org.fog.entities.FogBroker;
import org.fog.entities.PhysicalTopology;
import org.fog.entities.Tuple;
import org.fog.placement.Controller;
import org.fog.placement.ModuleMapping;
import org.fog.placement.ModulePlacementEdgewards;
import org.fog.utils.JsonToTopology;

/**
 * Simulation setup for Auto Car Driver extracting physical topology 
 * @author Manjunatha Gowda
 *
 */
public class autoCarDriver {

	public static void main(String[] args) {

		Log.printLine("Starting autoCarDriver...");

		try {
			Log.disable();
			int num_user = 1; // number of cloud users
			Calendar calendar = Calendar.getInstance();
			boolean trace_flag = false; // mean trace events

			CloudSim.init(num_user, calendar, trace_flag);

			String appId = "autoCarDriver";
			
			FogBroker broker = new FogBroker("broker");
			
			Application application = createApplication(appId, broker.getId());
			application.setUserId(broker.getId());
			
			/*
			 * Creating the physical topology from specified JSON file
			 */
			PhysicalTopology physicalTopology = JsonToTopology.getPhysicalTopology(broker.getId(), appId, "/iFogSim-main/iFogSim-main/src/topologies/CARTOPOLOGY1");

			ModuleMapping moduleMapping = ModuleMapping.createModuleMapping(); // initializing a module mapping

			
			moduleMapping.addModuleToDevice("store_d", "cloud");
			
			Controller controller = new Controller("master-controller", physicalTopology.getFogDevices(), physicalTopology.getSensors(), 
					physicalTopology.getActuators());
			
			controller.submitApplication(application, 0, new ModulePlacementEdgewards(physicalTopology.getFogDevices(), 
					physicalTopology.getSensors(), physicalTopology.getActuators(), 
					application, moduleMapping));
			
			CloudSim.startSimulation();

			CloudSim.stopSimulation();

			Log.printLine("autoCarDriver finished!");
		} catch (Exception e) {
			e.printStackTrace();
			Log.printLine("Unwanted errors happen");
		}
	}
	
	
	@SuppressWarnings({ "serial" })
	private static Application createApplication(String appId, int userId){
		
		Application application = Application.createApplication(appId, userId);
		
		application.addAppModule("process_d", 10);
		application.addAppModule("store_d", 10);
			
		application.addAppEdge("SENSOR", "process_d", 1000, 20000, "SENSOR", Tuple.UP, AppEdge.SENSOR); // adding edge from CAMERA (sensor) to Motion Detector module carrying tuples of type CAMERA
		application.addAppEdge("process_d", "store_d", 2000, 2000, "processed_data", Tuple.UP, AppEdge.MODULE); // adding edge from Motion Detector to Object Detector module carrying tuples of type MOTION_VIDEO_STREAM
		application.addAppEdge("process_d", "ACTUATOR", 100, 28, 100, "PTZ_PARAMS", Tuple.DOWN, AppEdge.ACTUATOR); // adding edge from Object Tracker to PTZ CONTROL (actuator) carrying tuples of type PTZ_PARAMS
//		application.addAppEdge("store_d", "ACTUATOR", 100, 28, 100, "PTZ_PARAMS2", Tuple.DOWN, AppEdge.ACTUATOR); // adding edge from Object Tracker to PTZ CONTROL (actuator) carrying tuples of type PTZ_PARAMS
		
		/*
		 * Defining the input-output relationships (represented by selectivity) of the application modules. 
		 */
		application.addTupleMapping("process_d", "SENSOR", "processed_data", new FractionalSelectivity(0.10)); // 1.0 tuples of type MOTION_VIDEO_STREAM are emitted by Motion Detector module per incoming tuple of type CAMERA
//		application.addTupleMapping("store_d", "processed_data", "PTZ_PARAMS2", new FractionalSelectivity(0.25)); // 0.05 tuples of type MOTION_VIDEO_STREAM are emitted by Object Detector module per incoming tuple of type MOTION_VIDEO_STREAM
		application.addTupleMapping("process_d", "SENSOR", "PTZ_PARAMS", new FractionalSelectivity(0.10)); // 1.0 tuples of type MOTION_VIDEO_STREAM are emitted by Motion Detector module per incoming tuple of type CAMERA
	
		/*
		 * Defining application loops (maybe incomplete loops) to monitor the latency of. 
		 * Here, we add two loops for monitoring : Motion Detector -> Object Detector -> Object Tracker and Object Tracker -> PTZ Control
		 */
		final AppLoop loop1 = new AppLoop(new ArrayList<String>(){{ add("SENSOR"); add("process_d");add("store_d");}});
		final AppLoop loop2 = new AppLoop(new ArrayList<String>(){{ add("SENSOR"); add("process_d");add("ACTUATOR");}});
		List<AppLoop> loops = new ArrayList<AppLoop>(){{add(loop1);add(loop2);}};

		application.setLoops(loops);
		
		//GeoCoverage geoCoverage = new GeoCoverage(-100, 100, -100, 100);
		return application;
	}
}