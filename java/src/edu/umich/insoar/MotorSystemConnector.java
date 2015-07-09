package edu.umich.insoar;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JMenu;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import probcog.arm.ArmStatus;
import probcog.lcmtypes.planner_command_t;
import probcog.lcmtypes.planner_response_t;
import sml.Agent;
import sml.Agent.OutputEventInterface;
import sml.Agent.RunEventInterface;
import sml.Identifier;
import sml.WMElement;
import sml.smlRunEventId;
import april.config.Config;
import april.config.ConfigFile;
import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.util.TimeUtil;

import com.soartech.bolt.script.ui.command.ResetRobotArm;

import edu.umich.insoar.world.Pose;
import edu.umich.insoar.world.SVSCommands;
import edu.umich.insoar.world.WMUtil;

public class MotorSystemConnector implements OutputEventInterface,
					     RunEventInterface,
					     LCMSubscriber {
    private SoarAgent agent;
    private Identifier inputLinkId;
    private Identifier selfId;
    private Identifier plannerId;

    private Pose pose;

    // Last received information about the arm

    private boolean gotUpdate = false;
    private boolean gotArmUpdate = false;
    private boolean spam = false;

    private String lastRequest = "NONE";
    private boolean requestFinished = false;
    private boolean requestSuccess = false;
    private int planSize = 0;

    private LCM lcm;

    private ArmStatus armStatus;

    StringBuilder svsCommands = new StringBuilder();

    private planner_command_t sentCommand = null;
    private long sentTime = 0;

    PerceptionConnector perception;

    public MotorSystemConnector(SoarAgent agent,
				PerceptionConnector perception) {
    	this.agent = agent;
    	pose = new Pose();

    	if (agent.getArmConfig() == null) {
    		armStatus = null;
    	} else {
	    try {
		 Config config = new ConfigFile(agent.getArmConfig());
		 armStatus = new ArmStatus(config);
	     } catch (IOException e) {
		 armStatus = null;
	     }
	 }

	 this.perception = perception;

	 // Setup LCM events
	 lcm = LCM.getSingleton();
	 lcm.subscribe("ARM_STATUS", this);
	 lcm.subscribe("PLANNER_RESPONSES", this);

	 // Setup Input Link Events
	 inputLinkId = agent.getAgent().GetInputLink();
	 agent.getAgent().RegisterForRunEvent(smlRunEventId.smlEVENT_BEFORE_INPUT_PHASE, this, null);

	 // Setup Output Link Events
	 String[] outputHandlerStrings = { "search",
					   "stop",
					   "execute",
					   "reset"};
	 for (String outputHandlerString : outputHandlerStrings)
	 {
		 agent.getAgent().AddOutputHandler(outputHandlerString, this, null);
	 }
     }

     @Override
     public synchronized void messageReceived(LCM lcm,
					      String channel,
					      LCMDataInputStream ins){
	 if(channel.equals("ARM_STATUS")){
	     gotArmUpdate = true;
	     if (spam) {
		 lcm.publish("PLANNER_COMMANDS", sentCommand);
	     }
	 }
	 if(channel.equals("PLANNER_RESPONSES")){
	     System.out.println("Got a planner response.");
            try {
                planner_response_t r = new planner_response_t(ins);
		if (r.response_type.equals("SEARCH") &&
		    lastRequest.equals("SEARCH") &&
		    r.finished) {
		    requestSuccess = r.success;
		    requestFinished = true;
		    planSize = r.plan_size;
		    gotUpdate = true;
		    spam = false;
		}
		if (r.response_type.equals("EXECUTE") &&
		    lastRequest.equals("EXECUTE") &&
		    r.finished) {
		    requestSuccess = r.success;
		    requestFinished = true;
		    gotUpdate = true;
		    spam = false;
		}
            }
            catch (IOException e){
                e.printStackTrace();
                return;
            }
	 }
     }

    // Happens during an input phase
    public synchronized void runEventHandler(int eventID,
					     Object data,
					     Agent agent, int phase){
    	long time = 0;
    	if(InSoar.DEBUG_TRACE){
	    time = TimeUtil.utime();
    	}

	if(selfId == null){
	    initIL();
	} else if(gotUpdate){
	    updateIL();
	    gotUpdate = false;
	}
	if(armStatus != null){
	    updateArmInfo();
	}
	if(svsCommands.length() > 0){
	    agent.SendSVSInput(svsCommands.toString());
	    //System.out.println(svsCommands.toString());
	    svsCommands = new StringBuilder();
	}
	this.agent.commitChanges();
    	if(InSoar.DEBUG_TRACE){
	    System.out.println(String.format("%-20s : %d", "MOTOR CONNECTOR", (TimeUtil.utime() - time)/1000));
    	}
    }

    private void initIL(){
    	selfId = inputLinkId.CreateIdWME("self");
	plannerId = inputLinkId.CreateIdWME("planner");
	plannerId.CreateStringWME("last-completed", "none");
	plannerId.CreateStringWME("success", "false");
	plannerId.CreateIntWME("plan-size", 0);

    	// selfId.CreateStringWME("action", "wait");
    	// selfId.CreateStringWME("prev-action", "wait");
    	// selfId.CreateStringWME("holding-obj", "false");
    	// selfId.CreateIntWME("grabbed-object", -1);
    	pose.updateWithArray(new double[]{0, 0, 0, 0, 0, 0});
    	pose.updateInputLink(selfId);

    	if(armStatus != null){
	    svsCommands.append("a arm object world p 0 0 0 r 0 0 0\n");
	    ArrayList<Double> widths = armStatus.getArmSegmentWidths();
	    ArrayList<double[]> points = armStatus.getArmPoints();
	    for(int i = 0; i < widths.size(); i++){
		// For each segment on the arm, initialize with the correct bounding volume
		String name = "seg" + i;
		double[] p1 = points.get(i);
		double[] p2 = points.get(i+1);
		double len = LinAlg.distance(p1, p2); 
		double[] size = new double[]{len, widths.get(i), widths.get(i)};
		if(i == widths.size()-1){
		    // Make the gripper bigger to help with occlusion checks;
		    size = LinAlg.scale(size, 2);
		}
		svsCommands.append("a " + name + " object arm p 0 0 0 r 0 0 0 ");
		svsCommands.append("s " + size[0] + " " + size[1] + " " + size[2] + " ");
		svsCommands.append("v " + SVSCommands.bboxVertices() + "\n");
	    }
    	}
    }

    private void updateIL(){
	WMUtil.updateStringWME(plannerId, "last-completed",
			       lastRequest.toLowerCase());
	if(requestSuccess) {
	    WMUtil.updateStringWME(plannerId, "success", "true");
	} else {
	    WMUtil.updateStringWME(plannerId, "success", "false");
	}
	if (lastRequest.equals("SEARCH")) {
	    WMUtil.updateIntWME(plannerId, "plan-size", planSize);
	}
    }

    private void updateArmInfo(){
    	if(!gotArmUpdate){
	    return;
    	}
    	gotArmUpdate = false;
    	ArrayList<Double> widths = armStatus.getArmSegmentWidths();
    	ArrayList<double[]> points = armStatus.getArmPoints();
    	for(int i = 0; i < widths.size(); i++){
	    String name = "seg" + i;

	    double[] p1 = points.get(i);
	    double[] p2 = points.get(i+1);
	    double[] center = LinAlg.scale(LinAlg.add(p1, p2), .5);
	    double[] dir = LinAlg.subtract(p2, p1);

	    double hyp = Math.sqrt(dir[0] * dir[0] + dir[1] * dir[1]);

	    double theta = 0;
	    if(Math.abs(dir[0]) > .0001 || Math.abs(dir[1]) > .0001){
		theta = Math.atan2(dir[1], dir[0]);
	    }

	    double phi = Math.PI/2;
	    if(Math.abs(hyp) > .0001 || Math.abs(dir[2]) > .0001){
		phi = -Math.atan2(dir[2], hyp);
	    }

	    double[][] rotZ = LinAlg.rotateZ(theta);
	    double[][] rotY = LinAlg.rotateY(phi);

	    double[] rot = LinAlg.matrixToRollPitchYaw(LinAlg.matrixAB(rotZ, rotY));

	    svsCommands.append(SVSCommands.changePos(name, center));
	    svsCommands.append(SVSCommands.changeRot(name, rot));
    	}
    }

    @Override
    public synchronized void outputEventHandler(Object data,
						String agentName,
						String attributeName,
						WMElement wme) {
	if (!(wme.IsJustAdded() && wme.IsIdentifier()))
        {
            return;
        }
	Identifier id = wme.ConvertToIdentifier();
        System.out.println(wme.GetAttribute());

        try{
            if (wme.GetAttribute().equals("search")) {
                processSearchCommand(id);
            }
            else if (wme.GetAttribute().equals("stop")) {
                processStopCommand(id);
            }
            else if (wme.GetAttribute().equals("execute")) {
                processExecuteCommand(id);
            }
	    else if (wme.GetAttribute().equals("reset")) {
		processResetCommand(id);
	    }
            agent.commitChanges();
        } catch (IllegalStateException e){
        	System.out.println(e.getMessage());
        }
    }

    private void processSearchCommand(Identifier id)
    {
        Identifier searchId =
	    WMUtil.getIdentifierOfAttribute(id, "target",
                "Error: No target location identifier");
        double x = Double.parseDouble(WMUtil.getValueOfAttribute(
                searchId, "x",
		"Error: No target x attribute"));
        double y = Double.parseDouble(WMUtil.getValueOfAttribute(
                searchId, "y",
		"Error: No target y attribute"));
        double z = Double.parseDouble(WMUtil.getValueOfAttribute(
                searchId, "z",
		"Error: No target z attribute"));

        double ss = Double.parseDouble(WMUtil.getValueOfAttribute(
                id, "step-size",
		"Error: No step-size attribute"));

        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "SEARCH";
	double[] t = {x, y, z};
        command.target = t;
	command.primitive_size = ss;
    	lcm.publish("PLANNER_COMMANDS", command);
        id.CreateStringWME("status", "searching");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "SEARCH";
	requestFinished = false;
	requestSuccess = false;
	spam = true;
    }

    private void processStopCommand(Identifier stopId)
    {
        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "STOP";
    	lcm.publish("PLANNER_COMMANDS", command);
        stopId.CreateStringWME("status", "killed");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "STOP";
	spam = true;
    }

    private void processExecuteCommand(Identifier id)
    {
        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "EXECUTE";
    	lcm.publish("PLANNER_COMMANDS", command);
        id.CreateStringWME("status", "executing");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "EXECUTE";
	requestFinished = false;
	requestSuccess = false;
	spam = true;
    }

    private void processResetCommand(Identifier id)
    {
        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "RESET";
    	lcm.publish("PLANNER_COMMANDS", command);
        id.CreateStringWME("status", "resetting");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "RESET";
	requestFinished = false;
	requestSuccess = false;
	spam = true;
    }

    public JMenu createMenu(){
    	JMenu actionMenu = new JMenu("Action");
    	JButton armResetButton  = new JButton("Reset Arm");
        armResetButton.addActionListener(new ActionListener(){
		public void actionPerformed(ActionEvent arg0) {
		    new ResetRobotArm().execute();
		}
	    });
        actionMenu.add(armResetButton);

        return actionMenu;
    }
}
