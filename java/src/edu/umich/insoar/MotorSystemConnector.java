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
import probcog.lcmtypes.robot_action_t;
import probcog.lcmtypes.robot_command_t;
import probcog.lcmtypes.set_state_command_t;
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

public class MotorSystemConnector   implements OutputEventInterface, RunEventInterface, LCMSubscriber{
	private SoarAgent agent;
    private Identifier inputLinkId;
	private Identifier selfId;

	private Pose pose;
	
	private robot_action_t curStatus = null;
	private robot_action_t prevStatus = null;
	// Last received information about the arm
	
	private boolean gotUpdate = false;
	
    private LCM lcm;
    
    private ArmStatus armStatus;
    
    StringBuilder svsCommands = new StringBuilder();

    public MotorSystemConnector(SoarAgent agent){
    	this.agent = agent;
    	pose = new Pose();
    	
    	if(agent.getArmConfig() == null){
    		armStatus = null;
    	} else {
	        try {
	  			Config config = new ConfigFile(agent.getArmConfig());
	  			armStatus = new ArmStatus(config);
	  		} catch (IOException e) {
	  			armStatus = null;
	  		}
    	}
    	
    	// Setup LCM events
        lcm = LCM.getSingleton();
        lcm.subscribe("ROBOT_ACTION", this);

        // Setup Input Link Events
        inputLinkId = agent.getAgent().GetInputLink();
        agent.getAgent().RegisterForRunEvent(smlRunEventId.smlEVENT_BEFORE_INPUT_PHASE, this, null);
        
        // Setup Output Link Events
        String[] outputHandlerStrings = { "move", "grasp", "release", "set-state"};
        for (String outputHandlerString : outputHandlerStrings)
        {
        	agent.getAgent().AddOutputHandler(outputHandlerString, this, null);
        }
    }
    
    @Override
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
    	if(channel.equals("ROBOT_ACTION")){
    		try {
    			robot_action_t action = new robot_action_t(ins);
				newRobotStatus(action);
			} catch (IOException e) {
				e.printStackTrace();
			}
    	}
    }
    
    public void newRobotStatus(robot_action_t status){
    	prevStatus = curStatus;
    	curStatus = status;
    	gotUpdate = true;
    }
    
    public String getStatus(){
    	if(curStatus == null){
    		return "wait";
    	} else {
    		return curStatus.action.toLowerCase();
    	}
    }

	// Happens during an input phase
	public synchronized void runEventHandler(int eventID, Object data, Agent agent, int phase){
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
	}
    
    private void initIL(){
    	selfId = inputLinkId.CreateIdWME("self");
    	selfId.CreateStringWME("action", "wait");
    	selfId.CreateStringWME("prev-action", "wait");
    	selfId.CreateStringWME("holding-obj", "false");
    	selfId.CreateIntWME("grabbed-object", -1);
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
        		
        		svsCommands.append("a " + name + " object arm p 0 0 0 r 0 0 0 ");
        		svsCommands.append("s " + len + " " + widths.get(i) + " " + widths.get(i) + " ");
        		svsCommands.append("v " + SVSCommands.bboxVertices() + "\n");
        	}
    	}
    }
    
    
    
    private void updateIL(){
    	if(prevStatus == null){
    		return;
    	}
    	WMUtil.updateStringWME(selfId, "action", curStatus.action.toLowerCase());
    	WMUtil.updateStringWME(selfId, "prev-action", prevStatus.action.toLowerCase());
    	WMUtil.updateStringWME(selfId, "holding-obj", (curStatus.obj_id != -1 ? "true" : "false"));
    	WMUtil.updateIntWME(selfId, "grabbed-object", curStatus.obj_id);
    	pose.updateWithArray(curStatus.xyz);
    	pose.updateInputLink(selfId);
    }
    
    private void updateArmInfo(){
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
    public synchronized void outputEventHandler(Object data, String agentName,
            String attributeName, WMElement wme) {
    	// Lizzie: Make sure what you did here doesn't cause horrible bugs for anyone
    	// else!
		if (!wme.IsJustAdded())
        {
            return;
        }
		Identifier id = wme.ConvertToIdentifier();
            
        try{
            if (wme.GetAttribute().equals("set-state")) {
                processSetCommand(id);
            } 
            else if (wme.GetAttribute().equals("move")) {
                processMoveCommand(id);
            } 
            else if (wme.GetAttribute().equals("grasp")) {
                processGraspCommand(id);
            } 
            else if (wme.GetAttribute().equals("release")) {
                processReleaseCommand(id);
            } 
            agent.commitChanges();
            System.out.println("[ROSIE] Sent " + wme.GetAttribute() + " command.");
        } catch (IllegalStateException e){
        	System.out.println(e.getMessage());
        }
	}
    
    /**
     * Takes a move command on the output link given as an identifier and
     * uses it to update the internal robot_command_t command. Expects move
     * ^pose <p> <p> ^x [float] ^y [float] ^z [float].
     */
    private void processMoveCommand(Identifier moveId)
    {
    	Identifier poseId = WMUtil.getIdentifierOfAttribute(
                moveId, "pose",
                "[OUTPUT] ERROR: Move command has no ^pose identifier");
        double x = Double.parseDouble(WMUtil.getValueOfAttribute(
                poseId, "x", "[OUTPUT] ERROR: Move pose has no ^x attribute"));
        double y = Double.parseDouble(WMUtil.getValueOfAttribute(
        		poseId, "y", "[OUTPUT] ERROR: Move pose has no ^y attribute"));
        double z = Double.parseDouble(WMUtil.getValueOfAttribute(
        		poseId, "z", "[OUTPUT] ERROR: Move pose has no ^z attribute"));
                                
        robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime();
        command.action = String.format("MOVE");
        command.dest = new double[]{x, y, z, 0, 0, 0};
        lcm.publish("ROBOT_COMMAND", command);
        moveId.CreateStringWME("status", "complete");
    }

    /**
	 * XXX Implement grasp...
     */
    private void processGraspCommand(Identifier graspId)
    {
    	robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime();
        command.action = String.format("GRASP");
        lcm.publish("ROBOT_COMMAND", command);
        
        graspId.CreateStringWME("status", "complete");
    }

    /**
	 * XXX Implement release...
     */
    private void processReleaseCommand(Identifier releaseId)
    {
        System.out.println("RELEASE: not implemented yet");
    }
    
    /**
     * Takes a set-state command on the output link given as an identifier and
     * uses it to update the internal robot_command_t command
     */
    private void processSetCommand(Identifier id)
    {
        String objId = WMUtil.getValueOfAttribute(id, "id",
                "Error (set-state): No ^id attribute");
        String name = WMUtil.getValueOfAttribute(id,
                "name", "Error (set-state): No ^name attribute");
        String value = WMUtil.getValueOfAttribute(id, "value",
                "Error (set-state): No ^value attribute");

        String action = String.format("ID=%s,%s=%s", objId, name, value);
        set_state_command_t command = new set_state_command_t();
        command.utime = TimeUtil.utime();
        command.state_name = name;
        command.state_val = value;
        command.obj_id = Integer.parseInt(objId);
    	lcm.publish("SET_STATE_COMMAND", command);
        id.CreateStringWME("status", "complete");
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
