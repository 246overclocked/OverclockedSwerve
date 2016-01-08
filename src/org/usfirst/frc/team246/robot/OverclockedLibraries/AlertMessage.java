/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team246.robot.OverclockedLibraries;

/**
 *
 * @author Dave
 */
public class AlertMessage {
    public enum Severity
    {
        CHITCHAT ("CHITCHAT"),
        DEBUG ("DEBUG"),
        WARNING ("WARNING"),
        ERROR ("ERROR"),
        FATAL ("FATAL");
        
        private final String name;
        
        private Severity(String s)
        {
            name = s;
        }
        
        public String toString()
        {
            return name;
        }
    }

    private String message;
    private String soundToPlay;
    private Severity severity;
    private String category;
    private String subsystem;
    private String details;
    
    public AlertMessage()
    {
        this.severity = Severity.WARNING;
    }
    
    public AlertMessage(String message)
    {
        this();
        this.message = message;
    }
    
    public AlertMessage message(String message)
    {
        this.message = message;
        return this;
    }
    
    public AlertMessage playSound(String soundToPlay)
    {
        this.soundToPlay = soundToPlay;
        return this;
    }
    
    public AlertMessage severity(Severity severity)
    {
        this.severity = severity;
        return this;
    }
    
    public AlertMessage category(String category)
    {
        this.category = category;
        return this;
    }
    
    public AlertMessage subsystem(String subsystem)
    {
        this.subsystem = subsystem;
        return this;
    }
    
    public AlertMessage addDetail(String details)
    {
        if (this.details == null) this.details = details;
        else this.details = this.details.concat("\n" + details);
        return this;
    }
    
    public String toXml()
    {
        StringBuilder sb = new StringBuilder();
        sb.append("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
        sb.append("<AlertMessage>\n");
        if (message != null)
        {
            sb.append("  <Message>");
            sb.append(message);
            sb.append("</Message>\n");
        }
        if (soundToPlay != null)
        {
            sb.append("  <SoundFileToPlay>");
            sb.append(soundToPlay);
            sb.append("</SoundFileToPlay>\n");
        }
        sb.append("  <SeverityLevel>");
        sb.append(severity.toString());
        sb.append("</SeverityLevel>\n");
        if (category != null)
        {
            sb.append("  <MessageCategory>");
            sb.append(category);
            sb.append("</MessageCategory>\n");
        }
        if (subsystem != null)
        {
            sb.append("  <SubsystemId>");
            sb.append(subsystem);
            sb.append("</SubsystemId>\n");
        }
        if (details != null)
        {
            sb.append("  <Details>");
            sb.append(details);
            sb.append("</Details>\n");
        }
        sb.append("</AlertMessage>");
        return sb.toString();
    }
}
