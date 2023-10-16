///////////////////////////////////////////////////////////////////////////////
//
//  Copyright Copyright (C) 2001-2021 Force Dimension, Switzerland.
//  All Rights Reserved.
//
//  drd 3.14.0
//  Force Dimension SDK
//
//  THIS FILE CAN NOT BE COPIED AND/OR DISTRIBUTED WITHOUT EXPRESS
//  PERMISSION FROM FORCE DIMENSION.
//
///////////////////////////////////////////////////////////////////////////////

import com.forcedimension.sdk.*;
import static java.lang.Math.PI;
import static java.lang.Math.sin;
import static java.lang.Math.cos;

// This is a Java implementation of the 'robot' example available 
// in the Force Dimension SDK/examples/CLI/robot/robot.cpp file.
class robot
{

public static void main(String[] args)
{
	final double[] zerovect = { 0.0, 0.0, 0.0 };

	double[] p = new double[3];
	double[] lim  = new double[10];
	double[] avj = new double[3];
	int done = 0;

	DHD dhd = new DHD();
	DRD drd = new DRD();

	// message
	System.out.println("");
	System.out.println("Force Dimension - Java (JNI) Robot Control Example  " + DHD.GetSDKVersionString());
	System.out.println("Copyright (C) 2001-2021 Force Dimension");
	System.out.println("All Rights Reserved.");
	System.out.println("");

	// open first available device
	if (drd.Open() < 0) 
	{
		System.out.println("error: cannot open device (" + DHD.GetLastErrorString() + ")");
		DHD.Sleep(2.0);
		return;
	}

	// assign ID to corresponding haptic device (to access haptic functions)
	dhd.SetDeviceID(drd.GetDeviceID());

	// print out device identifier
	if (!drd.IsSupported()) 
	{
		System.out.println("unsupported device");
		System.out.println("exiting...");
		DHD.Sleep(2.0);
		drd.Close();
		dhd.Close();
		return;
	}
	System.out.println(dhd.GetSystemName() + " haptic device detected");
	System.out.println("");

	// display instructions
	if (!drd.IsInitialized() && dhd.GetSystemType() == DHD.DEVICE_FALCON) 
	{
		System.out.printf("please initialize Falcon device...\r");
		while (!drd.IsInitialized()) dhd.SetForce (zerovect);
		System.out.printf("                                  \r");
		DHD.Sleep(0.5);
	}

	// initialize if necessary
	if (!drd.IsInitialized() && (drd.AutoInit() < 0)) 
	{
		System.out.println("error: initialization failed (" + DHD.GetLastErrorString() + ")");
		DHD.Sleep(2.0);
		return;
	}

	// start robot control loop
	if (drd.Start() < 0) 
	{
		System.out.println("error: control loop failed to start properly (" + DHD.GetLastErrorString() + ")");
		DHD.Sleep(2.0);
		return;
	}

	// get workspace limits
	for (int i=0; i<10; i++) lim[i] = -0.04 + i*(0.08/10.0);

	// DEMO PART 1: move within the cubic robotic workspace at default accel/speed/decel
	//              (which are conservative) and with default precision (10 um)

    				done = 0;
	while (done > -1 && done < 6) 
	{
		// give the control thread some slack
		DHD.Sleep(0.2);

		// generate random point within workspace
		while (!drd.IsMoving()) 
		{
			p[0] = 0.02 + lim[(int)(Math.random()*10.0)];
			p[1] =        lim[(int)(Math.random()*10.0)];
			p[2] =        lim[(int)(Math.random()*10.0)];
			if (drd.MoveToPos(p, false) >= 0) done++;
		}

		// check that the control loop is still running
		if (!drd.IsRunning ()) done = -2;

		// let user know what is going on
		System.out.printf("control running at %.3f kHz, moving to %+.2f, %+.3f, %+.2f      \r", drd.GetCtrlFreq(), p[0], p[1], p[2]);
	}


	// DEMO PART 2: move to the extremities of the cubic robotic workspace,
	//              this time significantly faster

	// change motion generator parameters
	avj[0] = 10.0;  // amax
	avj[1] = 10.0;  // vmax
	avj[2] = 50.0;  // jerk
	drd.SetPosMoveParam (avj);

	if (done > -1) done = 0;
	while (done > -1 && done < 10) 
	{
		// give the control thread some slack
		DHD.Sleep(0.2);

		// generate random point within workspace
		while (!drd.IsMoving()) 
		{
			p[0] = 0.02 + lim[(int)(Math.random()*10.0)];
			p[1] =        lim[(int)(Math.random()*10.0)];
			p[2] =        lim[(int)(Math.random()*10.0)];
			if (drd.MoveToPos(p, false) >= 0) done++;
		}

		// check that the control loop is still running
		if (!drd.IsRunning()) done = -2;

		// let user know what is going on
		System.out.printf("control running at %.3f kHz, moving to %+.2f, %+.3f, %+.2f      \r", drd.GetCtrlFreq(), p[0], p[1], p[2]);
	}

	// reset motion control parameters
	avj[0] =  1.0;  // amax
	avj[1] =  1.0;  // vmax
	avj[2] = -1.0;  // jerk
	drd.SetPosMoveParam (avj);


	// DEMO PART 3: track a sphere in real-time
	//              this shows how to asynchronously send the robot along a control trajectory
	//              the trajectory is interpolated along the target points according to a set of physical parameters
	//              such as max allowed acceleration, max allowed velocity and max allowed deceleration time
	//              these parameters are controlled by the drdSetPoseTrackParam() function

	double dt;
	double t0;
	double x0 = 0.02;
	double y0 = 0.0;
	double z0 = 0.0;
	double radius = 0.03;
	double alpha, beta;
	double radial_vel   = 2*PI;
	double vertical_vel = PI/10.0;

	// move to the top of the sphere
	p[0] = x0;
	p[1] = y0;
	p[2] = z0+radius;
	if (done > -1) drd.MoveToPos(p);

	// enable trajectory interpolator and adjust its parameters
	drd.EnableFilter (true);
	avj[0] =  2.0; // amax
	avj[1] =  1.0; // vmax
	avj[2] = -1.0; // jerk
	drd.SetPosTrackParam (avj);

	// start tracking a trajectory on the surface of the sphere
	// a new point along the trajectory is sent to the robotic controller at about 20 Hz
	// thanks to the trajectory interpolator, the movement is smooth

	t0 = DRD.GetTime();
	if (done > -1) done = 0;
	while (done == 0) 
	{
		// generate next sphere target point
		dt = DRD.GetTime()-t0;
		alpha = radial_vel * dt;
		beta = PI/2.0 + (vertical_vel * dt);
		p[0] = x0+radius*cos(alpha)*cos(beta);
		p[1] = y0+radius*sin(alpha)*cos(beta);
		p[2] = z0+radius*sin(beta);
		drd.TrackPos (p);
		if (beta > 5.0*PI/2.0)
		{
			done = -1;
		}

		// check for exit condition
		if (!drd.IsRunning())
		{
			done = -2;
		}

		// throttle trajectory updates to 20 Hz
		System.out.printf("control running at %.3f kHz, tracing sphere surface...     \r", drd.GetCtrlFreq());
		DRD.Sleep(0.05);
	}

	// report control loop errors
	if (done == -2) System.out.println("error: robot control thread aborted");

	// close the connection
	System.out.println("cleaning up...                                             ");
	drd.Close ();

	// happily exit
	System.out.println("");
	System.out.println("done.");

	return;
}

}  // class robot
