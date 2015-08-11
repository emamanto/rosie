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
    private Identifier lastRequestId;
    private Identifier idToUpdate;

    private Pose pose;

    // Last received information about the arm

    private boolean gotUpdate = false;
    private boolean gotArmUpdate = false;
    private boolean spam = false;

    private String lastRequest = "NONE";
    private String lastSearchType = "NONE";
    private String requestStatus = "";
    private boolean requestSuccess = false;
    private int planSize = 0;

    private LCM lcm;

    private ArmStatus armStatus;

    StringBuilder svsCommands = new StringBuilder();

    private planner_command_t sentCommand = null;
    private long sentTime = 0;
    private int ongoingSearch = -1;
    private int ongoingExecute = -1;
    private Identifier ongoingSearchId;
    private Identifier ongoingExecuteId;

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
	 String[] outputHandlerStrings = { "plan",
					   "stop",
					   "pause",
					   "continue",
					   "postprocess",
					   "execute",
					   "reset"};
	 for (String outputHandlerString : outputHandlerStrings)
	 {
		 agent.getAgent().AddOutputHandler(outputHandlerString, this, null);
	 }

	 new CommanderThread().start();
     }

    private synchronized int getNextMsgId()
    {
	if (sentCommand == null) return 0;
	return sentCommand.command_id + 1;
    }

     @Override
     public synchronized void messageReceived(LCM lcm,
					      String channel,
					      LCMDataInputStream ins){
	 if(channel.equals("ARM_STATUS")){
	     gotArmUpdate = true;
	 }
	 if(channel.equals("PLANNER_RESPONSES")){
            try {
                planner_response_t r = new planner_response_t(ins);
		if (r.response_type.equals("STOP") &&
		    r.response_id == ongoingSearch &&
		    r.finished) {
		    requestSuccess = r.success;
		    requestStatus = "stopped";
		    planSize = r.plan_size;
		    gotUpdate = true;
		    spam = false;
		    idToUpdate = ongoingSearchId;
		}
		else if (r.response_type.equals("PAUSE") &&
		    r.response_id == ongoingSearch &&
		    !r.finished) {
		    requestSuccess = r.success;
		    requestStatus = "paused";
		    planSize = r.plan_size;
		    gotUpdate = true;
		    spam = false;
		    idToUpdate = ongoingSearchId;
		}
		else if (r.response_type.equals("PLAN") &&
			 r.response_id == ongoingSearch &&
			 r.finished) {
		    requestSuccess = r.success;
		    requestStatus = "finished";
		    planSize = r.plan_size;
		    gotUpdate = true;
		    spam = false;
		    idToUpdate = ongoingSearchId;
		}
		else if ((r.response_type.equals("EXECUTE") ||
			  r.response_type.equals("RESET")) &&
			 r.response_id == ongoingExecute &&
			 r.finished) {
		    requestSuccess = r.success;
		    requestStatus = "finished";
		    planSize = r.plan_size;
		    gotUpdate = true;
		    spam = false;
		    idToUpdate = ongoingExecuteId;
		}
		else if (r.response_id == sentCommand.command_id &&
		    r.finished) {
		    requestSuccess = r.success;
		    requestStatus = "finished";
		    planSize = r.plan_size;
		    gotUpdate = true;
		    spam = false;
		    idToUpdate = lastRequestId;
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
	WMUtil.updateStringWME(idToUpdate, "status", requestStatus);
	if(requestSuccess) {
	    idToUpdate.CreateStringWME("success", "true");
	} else {
	    idToUpdate.CreateStringWME("success", "false");
	}
	idToUpdate.CreateIntWME("plan-size", planSize);
	idToUpdate = null;
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
            if (wme.GetAttribute().equals("plan")) {
                processPlanCommand(id);
            }
            else if (wme.GetAttribute().equals("stop")) {
                processStopCommand(id);
            }
            else if (wme.GetAttribute().equals("pause")) {
                processPauseCommand(id);
            }
            else if (wme.GetAttribute().equals("continue")) {
                processContinueCommand(id);
            }
            else if (wme.GetAttribute().equals("postprocess")) {
                processPostprocessCommand(id);
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

    private void processPlanCommand(Identifier id)
    {
	String type = WMUtil.getValueOfAttribute(id, "type");
        Identifier targetId =
	    WMUtil.getIdentifierOfAttribute(id, "target",
                "Error: No target location identifier");

	double lim = Double.parseDouble(WMUtil.getValueOfAttribute(id, "time-limit", "Error: Plan without time-limit."));
	String hard = WMUtil.getValueOfAttribute(id, "hard-time-limit");

        planner_command_t command = new planner_command_t();
        command.command_type = "PLAN";
	command.plan_type = type.toUpperCase();
	command.time_limit = lim;

	if (hard.equals("true")) {
	    command.hard_limit = true;
	}
	else {
	    command.hard_limit = false;
	}

	if (type.equals("grasp")) {
	    int id_val = Integer.parseInt(WMUtil.getValueOfAttribute(targetId, "id", "Error: Grasp without object id."));
	    command.target_object_id = id_val;
	}
	else if (type.equals("drop")) {
	    double x = Double.parseDouble(WMUtil.getValueOfAttribute(targetId, "x", "Error: Drop without target x."));
	    double y = Double.parseDouble(WMUtil.getValueOfAttribute(targetId, "y", "Error: Drop without target y."));
	    double z = Double.parseDouble(WMUtil.getValueOfAttribute(targetId, "z", "Error: Drop without target z."));
	    double[] xyz = {x, y, z};
	    command.target = xyz;
	}
	else {
	    double x = Double.parseDouble(WMUtil.getValueOfAttribute(targetId, "x", "Error: Move without target x."));
	    double y = Double.parseDouble(WMUtil.getValueOfAttribute(targetId, "y", "Error: Move without target y."));
	    double z = Double.parseDouble(WMUtil.getValueOfAttribute(targetId, "z", "Error: Move without target z."));
	    double[] xyz = {x, y, z};
	    command.target = xyz;
	}

        double ss = Double.parseDouble(WMUtil.getValueOfAttribute(
                id, "step-size",
		"Error: Plan without step-size."));

	command.primitive_size = ss;
	command.command_id = getNextMsgId();
	ongoingSearch = command.command_id;
	lastSearchType = command.plan_type;
    	lcm.publish("PLANNER_COMMANDS", command);
        id.CreateStringWME("status", "requested");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "PLAN";
	lastRequestId = id;
	ongoingSearchId = id;
	requestSuccess = false;
	requestStatus = "requested";
	spam = true;
    }

    private void processStopCommand(Identifier stopId)
    {
        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "STOP";
	command.plan_type = lastSearchType;
	command.command_id = getNextMsgId();
    	lcm.publish("PLANNER_COMMANDS", command);
        stopId.CreateStringWME("status", "requested");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "STOP";
	spam = true;
    }

    private void processPauseCommand(Identifier pauseId)
    {
        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "PAUSE";
	command.plan_type = lastSearchType;
	command.command_id = getNextMsgId();
    	lcm.publish("PLANNER_COMMANDS", command);
        pauseId.CreateStringWME("status", "requested");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "PAUSE";
	lastRequestId = pauseId;
	spam = true;
    }

    private void processContinueCommand(Identifier contId)
    {
        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "CONTINUE";
	command.plan_type = lastSearchType;
	command.command_id = getNextMsgId();
    	lcm.publish("PLANNER_COMMANDS", command);
        contId.CreateStringWME("status", "requested");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "CONTINUE";
	lastRequestId = contId;
	spam = true;
    }

    private void processPostprocessCommand(Identifier id)
    {
        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "POSTPROCESS";
	command.plan_type = lastSearchType;
	command.precision = 0.5;
	command.command_id = getNextMsgId();
    	lcm.publish("PLANNER_COMMANDS", command);
        id.CreateStringWME("status", "requested");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "POSTPROCESS";
	lastRequestId = id;
	spam = true;
    }

    private void processExecuteCommand(Identifier id)
    {
        double speed =
	    Double.parseDouble(WMUtil.getValueOfAttribute(id,
							  "speed",
							  "Error: No speed value"));

        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "EXECUTE";
	command.plan_type = lastSearchType;
	command.speed = speed;
	command.command_id = getNextMsgId();
	ongoingExecute = command.command_id;
    	lcm.publish("PLANNER_COMMANDS", command);
        id.CreateStringWME("status", "requested");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "EXECUTE";
	lastRequestId = id;
	ongoingExecuteId = id;
	requestStatus = "requested";
	requestSuccess = false;
	spam = true;
    }

    private void processResetCommand(Identifier id)
    {
        planner_command_t command = new planner_command_t();
        command.utime = TimeUtil.utime();
        command.command_type = "RESET";
	command.plan_type = "NONE";
	command.command_id = getNextMsgId();
	ongoingExecute = command.command_id;
    	lcm.publish("PLANNER_COMMANDS", command);
        id.CreateStringWME("status", "requested");
        sentCommand = command;
        sentTime = TimeUtil.utime();

	lastRequest = "RESET";
	lastRequestId = id;
	ongoingExecuteId = id;
	requestStatus = "requested";
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

    class CommanderThread extends Thread
    {
        public void run()
        {
	    while(true) {
		if (spam && sentCommand != null) {
		    synchronized(this) {
			lcm.publish("PLANNER_COMMANDS", sentCommand);
		    }
		}
		TimeUtil.sleep(100);
	    }
        }
    }
}
