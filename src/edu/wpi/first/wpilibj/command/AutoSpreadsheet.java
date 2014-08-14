/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.command;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Hashtable;

import org.usfirst.frc.team330.robot.commands.MarsRock;
import org.usfirst.frc.team330.robot.commands.Wait;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
 * $Log: AutoSpreadsheet.java,v $
 * Revision 1.7  2013-02-19 11:00:43  jross
 * add action item
 *
 * Revision 1.6  2013-02-19 03:48:07  jross
 * rewrite previous history comment to avoid erroneous action item.
 *
 * Revision 1.5  2013-02-18 04:33:24  jross
 * delete comment about crashing, fixed crashing
 *
 * Revision 1.4  2013-02-18 00:41:06  jross
 * try to fix crash when MarsRock called twice. needs to be tested
 *
 * Revision 1.3  2013-02-16 21:49:25  jross
 * Made AutoSpreadsheetCommand an interface instead of an absctract class so that implementing classes can be found
 *
 * Revision 1.2  2013-02-14 03:58:29  jross
 * update name of file
 *
 * Revision 1.1  2013-02-08 04:19:43  jross
 * Add AutoSpreadsheet
 *
 * Revision 1.5  2013-01-30 05:01:13  echan
 * Added the AutoSpreadsheetCommandGroup to command
 *
 * Revision 1.4  2013-01-30 04:24:35  jross
 * Add AutoSpreadsheetCommandGroup to let CommandGroups execute in a script
 *
 * Revision 1.3  2013-01-19 22:16:21  jross
 * Fix MarsRock crash
 *
 * Revision 1.2  2013-01-18 05:27:37  jross
 * Move close to finally blocks
 *
 * Revision 1.1  2013-01-01 19:53:50  jross
 * Import code from Beachbot2012JavaBeta Project
 *
 * Revision 1.6  2012-11-14 05:03:33  jross
 * don't add commands if not found. Remove debug statements
 *
 * Revision 1.5  2012-11-13 04:34:39  jross
 * Updates to fix crashes in AutoSpreadsheet. There are still some bugs
 *
 * Revision 1.4  2012-10-27 02:43:17  jross
 * Only read script before execution
 *
 * Revision 1.3  2012-10-24 03:27:58  jross
 * Add third parameter
 *
 * Revision 1.2  2012-10-21 22:12:18  jross
 * Use copy method so that multiple instances of the same command can be used. Fix NPEs
 *
 * Revision 1.1  2012-10-21 04:02:23  jross
 * Read autonomous scripts from csv file
 *
 */
/**
 *
 * @author joe
 */
public class AutoSpreadsheet {
//    FileReader file = null;
    BufferedReader reader = null;
//    final String defaultFilename = "2014AutoModesJava.csv";
    String filename;
    String commandPackage;
    SendableChooser autoChooser;   
    
    public AutoSpreadsheet(String filename, String commandPackage)
    {
    	this.filename = filename;
    	this.commandPackage = commandPackage;
        selectedAuto = new String();
//      System.out.println("begin of AutoSpreadsheet Constructor");
      autoChooser = new SendableChooser();
//      System.out.println(MarsRock.class.getName());
      autoChooser.addDefault("Mars Rock", new MarsRock());
//    System.out.println("end of AutoSpreadsheet Constructor");
    }
    
    public AutoSpreadsheet()
    {
    	this("2014AutoModesJava.csv", "org.usfirst.frc.team330.robot.commands");
    }
    
//    private Hashtable<String,Command> commandTable = new Hashtable<String, Command>();
//    
//    public void addCommand(Command command)
//    {
//        commandTable.put(command.getName().toUpperCase(), command);
//    }
//
//    public void addCommand(AutoSpreadsheetCommandGroup command)
//    {
//        commandTable.put(command.getName().toUpperCase(), command);
//    }
//    
//    public void addCommand(String commandName)
//    {
//    	try {
//			Object command = Class.forName(commandPackage + "." + commandName).newInstance();
//			if ((command instanceof AutoSpreadsheetCommand) || (command instanceof AutoSpreadsheetCommandGroup))
//			{
//				commandTable.put(commandName.toUpperCase(), (Command)command);
//			}
//			else
//			{
//				System.err.println(command.toString() + " is not an instance of AutospreadsheetCommand or AutoSpreadsheetCommandGroup");
//			}
//		} catch (ClassNotFoundException e) {
//			System.err.println("Can not find command: " + commandName + " in package: " + commandPackage);
//			e.printStackTrace();
//		} catch (InstantiationException e) {
//			System.err.println("Could not instantiate command: " + commandName + " in package: " + commandPackage);
//			e.printStackTrace();
//		} catch (IllegalAccessException e) {
//			System.err.println("Could not access command: " + commandName + " in package: " + commandPackage);
//			e.printStackTrace();
//		} 
//    }


    public CommandGroup getSelected() {
        checkForChange();
        if (currentCommandGroup != null)
            return currentCommandGroup;
        else
            return buildScript((String)autoChooser.getSelected());
    }
            
    public void readScripts()
    {
        try {
            reader = new BufferedReader(new FileReader(filename));
           
            String line;
            String scriptName = null;
            int comma1, comma2;
            while ((line = reader.readLine()) != null)
            {
//                System.out.println(line);
                if (line.toUpperCase().startsWith("SCRIPT_NAME,"))
                {
                    comma1 = line.indexOf(",");
                    comma2 = line.substring(comma1+1).indexOf(",")+comma1+1;
                    scriptName = line.substring(comma1+1,comma2).toUpperCase().trim();
                    System.out.println("Found Script: " + scriptName);
                    autoChooser.addObject(scriptName, scriptName);
                }
                else if (line.toUpperCase().startsWith("END_OF_SPREAD_SHEET,"))
                {
//                        System.out.println("End of Spreadsheet");
                        break;                     
                }
            }           
            SmartDashboard.putData("Autonomous mode", autoChooser);
            
        } catch (IOException ex) {
            ex.printStackTrace();
        }
        finally
        {
            try {
                reader.close();
            } catch (IOException ex) {
                ex.printStackTrace();
            }
            currentCommandGroup = null;
        }
        currentCommandGroup = null;
    }
    
    public CommandGroup buildScript(String scriptToRead)
    {
        CommandGroup cg = null;
        try {
            reader = new BufferedReader(new FileReader(filename));
           
            String line;
            String scriptName = null;
            int comma1, comma2;
            boolean scriptStarted = false;
            String commandName = null;
            double timeout, param1, param2, param3;
            boolean stop;
            boolean sequential;
            

            Command command = null;
              
            //search for specified script name
            while ((line = reader.readLine()) != null)
            {
//                System.out.println(line);
                if (line.toUpperCase().startsWith("SCRIPT_NAME,"))
                {
                    comma1 = line.indexOf(",");
                    comma2 = line.substring(comma1+1).indexOf(",")+comma1+1;
                    scriptName = line.substring(comma1+1,comma2).toUpperCase().trim();
                    System.out.println("Found Script: " + scriptName);
                    scriptStarted = true;
                    if (scriptName.equals(scriptToRead))
                    {
                        cg = new CommandGroup(scriptName);
                        break;
                    }
                }
            }
            
            while ((line = reader.readLine()) != null)
            {
                if (line.toUpperCase().startsWith("END_OF_SPREAD_SHEET,"))
                {
                    if (scriptStarted)
                        System.err.println("!!!End of Spreadsheet found while " + scriptName + " still open");
                    else
                    {
//                        System.out.println("End of Spreadsheet");
                        break;
                    }
                }
                else if (line.toUpperCase().startsWith("END,"))
                {
//                    System.out.println("End of Script: " + scriptName);
                    scriptStarted = false;
                    cg.addSequential(new Wait(15));
                    break;
                }
                else if (line.startsWith(",")|| line.length() <= 2 )
                {
                    //empty line
                }
                else
                {
                    if (!scriptStarted)
                        System.err.println("!!!Command found while Script not open");
                    else
                    {
                        commandName = line.substring(0, line.indexOf(","));
                        
                        comma1 = line.indexOf(",");
                        comma2 = line.substring(comma1+1).indexOf(",")+comma1+1;
                        if (line.substring(comma1+1).startsWith("S"))
                            sequential = true;
                        else
                            sequential = false;
                        
                        line = line.substring(comma1+1);
                        comma1 = line.indexOf(",");
                        comma2 = line.substring(comma1+1).indexOf(",")+comma1+1;
                        try
                        {
                            timeout = Double.parseDouble(line.substring(comma1+1,comma2).toUpperCase().trim());
                        }
                        catch (NumberFormatException ex)
                        {
                            timeout = 0;
                        }

                        line = line.substring(comma2+1);
                        if (line.toUpperCase().startsWith("CONTINUE"))
                            stop = false;
                        else
                            stop = true;
//                        System.out.println("stop: " + stop + " " + line.toUpperCase());
                        
                        comma1 = line.indexOf(",");
                        comma2 = line.substring(comma1+1).indexOf(",")+comma1+1;
                        try
                        {
                            param1 = Double.parseDouble(line.substring(comma1+1,comma2).trim());  
                        }
                        catch (NumberFormatException ex)
                        {
                            param1 = 0;
                        }
                        
                        comma1 = comma2;
                        comma2 = line.substring(comma1+1).indexOf(",")+comma1+1;
                        try
                        {
                            param2 = Double.parseDouble(line.substring(comma1+1,comma2).trim());  
                        }
                        catch (NumberFormatException ex)
                        {
                            param2 = 0;
                        }
                        
                        comma1 = comma2;
                        comma2 = line.substring(comma1+1).indexOf(",")+comma1+1;
                        try
                        {
                            param3 = Double.parseDouble(line.substring(comma1+1,comma2).trim());  
                        }
                        catch (NumberFormatException ex)
                        {
                            param3 = 0;
                        }                       
                        
                    	try {
                			Object commandClass = Class.forName(commandPackage + "." + commandName).newInstance();
                			if ((commandClass instanceof AutoSpreadsheetCommand) || (commandClass instanceof CommandGroup))
                			{
                				command = (Command)commandClass;
                				if (command instanceof AutoSpreadsheetCommand)
                                {
                                    System.out.println("Found Command: " + command.getName());
//                                    command = ((AutoSpreadsheetCommand)command).copy();
                                    ((AutoSpreadsheetCommand)command).setStopAtEnd(stop);
                                    ((AutoSpreadsheetCommand)command).setParam1(param1);
                                    ((AutoSpreadsheetCommand)command).setParam2(param2);
                                    ((AutoSpreadsheetCommand)command).setParam3(param3);
                                    (command).setTimeout(timeout);
                                    if (sequential)   
                                        cg.addSequential(command);
                                    else
                                        cg.addParallel(command);
                                    
                                }
                                else if (command instanceof CommandGroup)
                                {
                                    System.out.println("Found Command Group: " + command.getName());
//                                    command = ((AutoSpreadsheetCommandGroup)command).copy();
                                    if (sequential)   
                                        cg.addSequential(command);
                                    else
                                        cg.addParallel(command);
                                }
                			}
                			else
                			{
                				System.err.println(commandClass.toString() + " is not an instance of AutospreadsheetCommand or AutoSpreadsheetCommandGroup");
                			}
                		} catch (ClassNotFoundException e) {
                			System.err.println("Can not find command: " + commandName + " in package: " + commandPackage);
                			e.printStackTrace();
                		} catch (InstantiationException e) {
                			System.err.println("Could not instantiate command: " + commandName + " in package: " + commandPackage);
                			e.printStackTrace();
                		} catch (IllegalAccessException e) {
                			System.err.println("Could not access command: " + commandName + " in package: " + commandPackage);
                			e.printStackTrace();
                		} 
                        
//                        command = (Command)commandTable.get(commandName.toUpperCase());
                       
                        
//                        if (command == null)
//                        {
//                            System.err.println("Could not find command: " + commandName);
//                        }
//                        else if (command instanceof AutoSpreadsheetCommand)
//                        {
//                            System.out.println("Found Command: " + command.getName());
//                            command = ((AutoSpreadsheetCommand)command).copy();
//                            ((AutoSpreadsheetCommand)command).setStopAtEnd(stop);
//                            ((AutoSpreadsheetCommand)command).setParam1(param1);
//                            ((AutoSpreadsheetCommand)command).setParam2(param2);
//                            ((AutoSpreadsheetCommand)command).setParam3(param3);
//                            (command).setTimeout(timeout);
//                            if (sequential)   
//                                cg.addSequential(command);
//                            else
//                                cg.addParallel(command);
//                            
//                        }
//                        else if (command instanceof AutoSpreadsheetCommandGroup)
//                        {
//                            System.out.println("Found Command Group: " + command.getName());
//                            command = ((AutoSpreadsheetCommandGroup)command).copy();
//                            if (sequential)   
//                                cg.addSequential(command);
//                            else
//                                cg.addParallel(command);
//                        }
//                        else
//                        {
//                            System.err.println(commandName + " (" + command.getName() + ") is not instance of AutoSpreadsheetCommand");
//                        }

                        
//                        System.out.println("Command: " + commandName + " Timeout: " + timeout + " Continue: " + !stop + " Param1: " + param1 + " Param2: " + param2 + " Param3: " + param3);
                    }
                }
            }
           
        } catch (IOException ex) {
            ex.printStackTrace();
        }
        finally
        {
            try {
                reader.close();
            } catch (IOException ex) {
                ex.printStackTrace();
            }
        }
        return cg;
    }
    
    String selectedAuto;
    CommandGroup currentCommandGroup = null;
    
    public void checkForChange() {
        if (autoChooser.getSelected() instanceof MarsRock) {
            CommandGroup cg = new CommandGroup();
            cg.addSequential(new MarsRock());
            currentCommandGroup = cg;
            return;
        }

        if(!selectedAuto.equalsIgnoreCase((String)autoChooser.getSelected())) {
            selectedAuto = (String)autoChooser.getSelected();
            currentCommandGroup = buildScript(selectedAuto);
            System.out.println("Selected Auto: " + selectedAuto);
        }
    }
}
