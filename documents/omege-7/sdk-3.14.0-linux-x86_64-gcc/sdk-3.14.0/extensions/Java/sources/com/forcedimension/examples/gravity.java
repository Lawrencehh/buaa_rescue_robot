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

import com.forcedimension.sdk.DHD;

// This is a Java implementation of the 'gravity' example available 
// in the Force Dimension SDK/examples/CLI/gravity/gravity.cpp file.
class gravity
{
	static private final double REFRESH_INTERVAL = 0.1;

	public static void main(String[] args)
	{
		final double[] zerovect = { 0.0, 0.0, 0.0 };

		double[] p = new double[3];
		double[] f = new double[3];
		double t1, t0 = DHD.GetTime();
		int done = 0;

		DHD dhd = new DHD();

		// message
		System.out.println("");
		System.out.println("Force Dimension - Java (JNI) Gravity Compensation Example " + DHD.GetSDKVersionString());
		System.out.println("Copyright (C) 2001-2021 Force Dimension");
		System.out.println("All Rights Reserved.");
		System.out.println("");

		// open the first available device
		if (dhd.Open() < 0) 
		{
			System.out.println("error: cannot open device (" + DHD.GetLastErrorString() + ")");
			DHD.Sleep(2.0);
			return;
		}

		// identify device
		System.out.println(dhd.GetSystemName() + " device detected");
		System.out.println("");

		// display instructions
		System.out.println("press BUTTON to quit");
		System.out.println("");

		// enable force
		dhd.EnableForce(true);
		while (done == 0) 
		{
			// apply zero force
			if (dhd.SetForceAndTorqueAndGripperForce(zerovect,zerovect,0.0) < DHD.NO_ERROR) 
			{
				System.out.println("error: cannot set force (" + DHD.GetLastErrorString() + ")");
				done = 1;
			}

			// display refresh rate and position at 10Hz
			t1 = DHD.GetTime();
			if ((t1-t0) > REFRESH_INTERVAL) 
			{
				// retrieve position
				if (dhd.GetPosition(p) < DHD.NO_ERROR) 
				{
					System.out.println("error: cannot read position (" + DHD.GetLastErrorString() + ")");
					done = 1;
				}

				// retrieve force
				if (dhd.GetForce(f) < DHD.NO_ERROR) 
				{
					System.out.println("error: cannot read force (" + DHD.GetLastErrorString() + ")");
					done = 1;
				}

				// display status
				System.out.printf("p (%+.3f %+.3f %+.3f) m  |  f (%+.1f %+.1f %+.1f) N  |  freq %.2f kHz\r", p[0], p[1], p[2], f[0], f[1], f[2], dhd.GetComFreq());

				// test for exit condition
				if (dhd.GetButton(0) > 0)
				{
					done = 1;
				}

				// update timestamp
				t0 = t1;
			}
		}

		// close the connection
		dhd.Close();

		// happily exit
		System.out.println("");
		System.out.println("done.");
	}

}  // class gravity
