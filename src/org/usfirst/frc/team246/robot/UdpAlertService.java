/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team246.robot;

import java.io.IOException;
import java.net.InetAddress;
import java.net.DatagramSocket;
import java.net.DatagramPacket;

/**
 *
 * @author Dave
 */
public class UdpAlertService {

    public static void initialize(String hostname, int port)
    {
    	UdpAlertService.alertServiceHost = hostname; 
    	UdpAlertService.alertServicePort = port;
    	initialize();
    }
    
    private static void initialize()
    {
    	try
    	{
	        alertServerAddress = InetAddress.getByName(alertServiceHost);
	        transmitSocket = new DatagramSocket();
	        transmitSocket.setBroadcast(false);
	        transmitSocket.setReuseAddress(true);
	        isInitialized = true;
    	}
    	catch (Exception e){}
    }
    
    private static InetAddress alertServerAddress;
    private static int alertServicePort;
    private static DatagramSocket transmitSocket;
    private static String alertServiceHost;
    private static boolean isInitialized = false;
    
    public static void sendAlert(AlertMessage alertMessage)
    {
//    	if (!isInitialized) initialize();
    	if (!isInitialized) return;
        String xmlString = alertMessage.toXml();
        DatagramPacket packet = new DatagramPacket(xmlString.getBytes(), xmlString.length(), alertServerAddress, alertServicePort);
        try {
			transmitSocket.send(packet);
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
    
    public static void close()
    {
    	if (!isInitialized) transmitSocket.close();
    }
}
