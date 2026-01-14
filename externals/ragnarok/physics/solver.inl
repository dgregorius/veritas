//--------------------------------------------------------------------------------------------------
// solver.inl	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkCompliance
//--------------------------------------------------------------------------------------------------
inline RkCompliance rkCreateCompliance( float Frequency, float DampingRatio, float Timestep )
	{
	RK_ASSERT( Frequency > 0.0f && DampingRatio > 0.0f );
	
	float Omega = RK_2PI * Frequency;
	float Zeta = DampingRatio;
	float C = Timestep * Omega * ( 2.0f * Zeta + Timestep * Omega );

	RkCompliance Compliance;
	Compliance.BiasScale = Timestep * Omega * Omega / C;
	Compliance.LambdaScale = 1.0f / ( 1.0f + C );
	Compliance.MassScale = C * Compliance.LambdaScale;

	return Compliance;
	}


