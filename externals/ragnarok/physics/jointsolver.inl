//--------------------------------------------------------------------------------------------------
// jointsolver.inl	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// Swing/Twist Decomposition
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkSwingAngle( const RkQuaternion& RelQ )
	{
	// Note: We assume the relative quaternion is build from two quaternions in the same hemisphere!
	float Swing = 2.0f * rkATan2( rkSqrt( RelQ.X * RelQ.X + RelQ.Y * RelQ.Y ), rkSqrt( RelQ.Z * RelQ.Z + RelQ.W * RelQ.W ) );
	RK_ASSERT( 0.0f <= Swing && Swing <= RK_PI );

	return Swing;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkTwistAngle( const RkQuaternion& RelQ )
	{
	// Note: We assume the relative quaternion is build from two quaternions in the same hemisphere!
	float Twist = 2.0f * rkATan2( RelQ.Z, RelQ.W );
	RK_ASSERT( -RK_PI <= Twist && Twist <= RK_PI );

	return Twist;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE float rkUnwind( float Angle )
	{
	RK_ASSERT( -RK_2PI <= Angle && Angle <= RK_2PI );
	if ( Angle < -RK_PI )
		{
		Angle += RK_2PI;
		}
	if ( Angle > RK_PI )
		{
		Angle -= RK_2PI;
		}

	return Angle;
	}


//--------------------------------------------------------------------------------------------------
// RkLinearConstraint2
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkLinearConstraint2& Constraint, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, RkVector3 Axes[ 2 ], const RkVector2& Bias, const RkVector2& Lambda )
	{
	float InvM1 = Body1->GetMassInv();
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	float InvM2 = Body2->GetMassInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();
	float InvM = InvM1 + InvM2;

	RkVector3 R1_x_S = rkCross( Offset1, Axes[ 0 ] );
	RkVector3 R2_x_S = rkCross( Offset2, Axes[ 0 ] );
	RkVector3 R1_x_T = rkCross( Offset1, Axes[ 1 ] );
	RkVector3 R2_x_T = rkCross( Offset2, Axes[ 1 ] );

	RkMatrix2 K;
	K.A11 = InvM + rkDot( R1_x_S, InvI1 * R1_x_S ) + rkDot( R2_x_S, InvI2 * R2_x_S );
	K.A22 = InvM + rkDot( R1_x_T, InvI1 * R1_x_T ) + rkDot( R2_x_T, InvI2 * R2_x_T );
	K.A12 = K.A21 = rkDot( R1_x_S, InvI1 * R1_x_T ) + rkDot( R2_x_S, InvI2 * R2_x_T );

	Constraint.Offset1 = Offset1;
	Constraint.Offset2 = Offset2;
	Constraint.Axes[ 0 ] = Axes[ 0 ];
	Constraint.Axes[ 1 ] = Axes[ 1 ];
	Constraint.Bias = Bias;
	Constraint.Mass = rkInvertT( K );
	Constraint.Lambda = Lambda;
	
	Constraint.InvM1 = InvM1;
	Constraint.W_JT1[ 0 ] = InvI1 * R1_x_S;
	Constraint.W_JT1[ 1 ] = InvI1 * R1_x_T;
	Constraint.InvM2 = InvM2;
	Constraint.W_JT2[ 0 ] = InvI2 * R2_x_S;
	Constraint.W_JT2[ 1 ] = InvI2 * R2_x_T;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkLinearConstraint2& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2, const RkCompliance& Compliance )
	{
	RkVector3 U1 = V1 + rkCross( W1, Constraint.Offset1 );
	RkVector3 U2 = V2 + rkCross( W2, Constraint.Offset2 );
	RkVector2 RelV( rkDot( U2 - U1, Constraint.Axes[ 0 ] ), rkDot( U2 - U1, Constraint.Axes[ 1 ] ) );

	RkVector2 Lambda = Constraint.Lambda;
	RkVector2 DeltaLambda = -Compliance.MassScale * Constraint.Mass * ( RelV + Compliance.BiasScale * Constraint.Bias ) - Compliance.LambdaScale * Lambda;
	Constraint.Lambda += DeltaLambda;

	RkVector3 LinearImpulse = DeltaLambda[ 0 ] * Constraint.Axes[ 0 ] + DeltaLambda[ 1 ] * Constraint.Axes[ 1 ];

	V1 -= Constraint.InvM1 * LinearImpulse;
	W1 -= Constraint.W_JT1[ 0 ] * DeltaLambda[ 0 ] + Constraint.W_JT1[ 1 ] * DeltaLambda[ 1 ];
	V2 += Constraint.InvM2 * LinearImpulse;
	W2 += Constraint.W_JT2[ 0 ] * DeltaLambda[ 0 ] + Constraint.W_JT2[ 1 ] * DeltaLambda[ 1 ];
	}


//--------------------------------------------------------------------------------------------------
// RkLinearConstraint3
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkLinearConstraint3& Constraint, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, const RkVector3& Bias, const RkVector3& Lambda )
	{
	float InvM1 = Body1->GetMassInv();
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	RkMatrix3 Skew1 = rkSkew( Offset1 );

	float InvM2 = Body2->GetMassInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();
	RkMatrix3 Skew2 = rkSkew( Offset2 );

	float InvM = InvM1 + InvM2;
	RkMatrix3 MassInv = RkMatrix3( InvM, InvM, InvM ) - Skew1 * InvI1 * Skew1 - Skew2 * InvI2 * Skew2;
	RkMatrix3 Mass = rkInvertT( MassInv );

	Constraint.Offset1 = Offset1;
	Constraint.Offset2 = Offset2;
	Constraint.Mass = Mass;
	Constraint.Bias = Bias;
	Constraint.Lambda = Lambda;
	
	Constraint.InvM1 = InvM1;
	Constraint.InvM2 = InvM2;
	Constraint.InvI1 = InvI1;
	Constraint.InvI2 = InvI2;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkLinearConstraint3& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2, const RkCompliance& Compliance )
	{
	RkVector3 U1 = V1 + rkCross( W1, Constraint.Offset1 );
	RkVector3 U2 = V2 + rkCross( W2, Constraint.Offset2 );
	RkVector3 RelV = U2 - U1;

	RkVector3 Lambda = Constraint.Lambda;
	RkVector3 DeltaLambda = -Compliance.MassScale * Constraint.Mass * ( RelV + Compliance.BiasScale * Constraint.Bias ) - Compliance.LambdaScale * Lambda;
	Constraint.Lambda += DeltaLambda;

	V1 -= Constraint.InvM1 * DeltaLambda;
	W1 -= Constraint.InvI1 * rkCross( Constraint.Offset1, DeltaLambda );
	V2 += Constraint.InvM2 * DeltaLambda;
	W2 += Constraint.InvI2 * rkCross( Constraint.Offset2, DeltaLambda );
	}


//--------------------------------------------------------------------------------------------------
// RkSoftLinearConstraint3
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkSoftLinearConstraint3& Constraint, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, const RkVector3& Bias, const RkVector3& Lambda, float MaxLambda, const RkCompliance& Compliance )
	{
	float InvM1 = Body1->GetMassInv();
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	RkMatrix3 Skew1 = rkSkew( Offset1 );

	float InvM2 = Body2->GetMassInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();
	RkMatrix3 Skew2 = rkSkew( Offset2 );

	float InvM = InvM1 + InvM2;
	RkMatrix3 MassInv = RkMatrix3( InvM, InvM, InvM ) - Skew1 * InvI1 * Skew1 - Skew2 * InvI2 * Skew2; 
	RkMatrix3 Mass = rkInvertT( MassInv );

	Constraint.Offset1 = Offset1;
	Constraint.Offset2 = Offset2;
	Constraint.BiasScale = Compliance.BiasScale;
	Constraint.Bias = Bias;
	Constraint.MassScale = Compliance.MassScale;
	Constraint.Mass = Mass;
	Constraint.LambdaScale = Compliance.LambdaScale;
	Constraint.Lambda = Lambda;
	Constraint.MaxLambda = MaxLambda;
	
	Constraint.InvM1 = InvM1;
	Constraint.InvM2 = InvM2;
	Constraint.InvI1 = InvI1;
	Constraint.InvI2 = InvI2;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkSoftLinearConstraint3& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 ) 
	{
	RkVector3 U1 = V1 + rkCross( W1, Constraint.Offset1 );
	RkVector3 U2 = V2 + rkCross( W2, Constraint.Offset2 );
	RkVector3 RelV = U2 - U1;

	RkVector3 Lambda = Constraint.Lambda;
	RkVector3 DeltaLambda = -Constraint.MassScale * Constraint.Mass * ( RelV + Constraint.BiasScale * Constraint.Bias ) - Constraint.LambdaScale * Lambda;
	Constraint.Lambda += DeltaLambda;

	float LambdaSq = rkDot( Constraint.Lambda, Constraint.Lambda );
	if ( LambdaSq > Constraint.MaxLambda * Constraint.MaxLambda )
		{
		Constraint.Lambda *= Constraint.MaxLambda / rkSqrt( LambdaSq );
		}
	DeltaLambda = Constraint.Lambda - Lambda;

	V1 -= Constraint.InvM1 * DeltaLambda;
	W1 -= Constraint.InvI1 * rkCross( Constraint.Offset1, DeltaLambda );
	V2 += Constraint.InvM2 * DeltaLambda;
	W2 += Constraint.InvI2 * rkCross( Constraint.Offset2, DeltaLambda );
	}


//--------------------------------------------------------------------------------------------------
// RkLinearLimit
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkLinearLimit& Constraint, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, const RkVector3& Axis, float Bias, float Lambda, float MinLambda, float MaxLambda, const RkCompliance& Compliance )
	{
	float InvM1 = Body1->GetMassInv();
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	float InvM2 = Body2->GetMassInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();

	RkVector3 R1_x_N = rkCross( Offset1, Axis );
	RkVector3 R2_x_N = rkCross( Offset2, Axis );

	float MassInv = InvM1 + InvM2 + rkDot( R1_x_N, InvI1 * R1_x_N ) + rkDot( R2_x_N, InvI2 * R2_x_N );
	float Mass = MassInv * MassInv > 1000.0f * RK_F32_MIN ? 1.0f / MassInv : 0.0f;

	Constraint.Offset1 = Offset1;
	Constraint.Offset2 = Offset2;
	Constraint.Axis = Axis;
	Constraint.BiasScale = Compliance.BiasScale;
	Constraint.Bias = Bias;
	Constraint.MassScale = Compliance.MassScale;
	Constraint.Mass = Mass;
	Constraint.LambdaScale = Compliance.LambdaScale;
	Constraint.Lambda = Lambda;
	Constraint.MinLambda = MinLambda;
	Constraint.MaxLambda = MaxLambda;
	
	Constraint.InvM1 = InvM1;
	Constraint.InvM2 = InvM2;
	Constraint.W_JT1 = InvI1 * R1_x_N;
	Constraint.W_JT2 = InvI2 * R2_x_N;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkLinearLimit& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 )
	{
	RkVector3 U1 = V1 + rkCross( W1, Constraint.Offset1 );
	RkVector3 U2 = V2 + rkCross( W2, Constraint.Offset2 );
	float RelV = rkDot( Constraint.Axis, U2 - U1 );

	float Lambda = Constraint.Lambda;
	float DeltaLambda = -Constraint.MassScale * Constraint.Mass * ( RelV + Constraint.BiasScale * Constraint.Bias ) - Constraint.LambdaScale * Lambda;
	Constraint.Lambda = rkClamp( Lambda + DeltaLambda, Constraint.MinLambda, Constraint.MaxLambda );
	DeltaLambda = Constraint.Lambda - Lambda;
	
	V1 -= ( Constraint.InvM1 * DeltaLambda ) * Constraint.Axis;
	W1 -= Constraint.W_JT1 * DeltaLambda;
	V2 += ( Constraint.InvM2 * DeltaLambda ) * Constraint.Axis;
	W2 += Constraint.W_JT2 * DeltaLambda;
	}


//--------------------------------------------------------------------------------------------------
// RkAngularConstraint2
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkAngularConstraint2& Constraint, const RkBody* Body1, const RkBody* Body2, RkVector3 Axes[ 2 ], const RkVector2& Bias, const RkVector2& Lambda )
	{
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();
	RkMatrix3 InvI = InvI1 + InvI2;

	RkMatrix2 K;
	K.A11 = rkDot( Axes[ 0 ], InvI * Axes[ 0 ] );
	K.A22 = rkDot( Axes[ 1 ], InvI * Axes[ 1 ] );
	K.A12 = K.A21 = rkDot( Axes[ 0 ], InvI * Axes[ 1 ] );

	Constraint.Axes[ 0 ] = Axes[ 0 ];
	Constraint.Axes[ 1 ] = Axes[ 1 ];
	Constraint.Bias = Bias;
	Constraint.Mass = rkInvertT( K );
	Constraint.Lambda = Lambda;
	
	Constraint.W_JT1[ 0 ] = InvI1 * Axes[ 0 ];
	Constraint.W_JT1[ 1 ] = InvI1 * Axes[ 1 ];
	Constraint.W_JT2[ 0 ] = InvI2 * Axes[ 0 ];
	Constraint.W_JT2[ 1 ] = InvI2 * Axes[ 1 ];
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkAngularConstraint2& Constraint, RkVector3& W1, RkVector3& W2, const RkCompliance& Compliance )
	{
	RkVector2 RelV( rkDot( W2 - W1, Constraint.Axes[ 0 ] ), rkDot( W2 - W1, Constraint.Axes[ 1 ] ) );

	RkVector2 Lambda = Constraint.Lambda;
	RkVector2 DeltaLambda = -Compliance.MassScale * Constraint.Mass * ( RelV + Compliance.BiasScale * Constraint.Bias ) - Compliance.LambdaScale * Lambda;
	Constraint.Lambda += DeltaLambda;

	W1 -= Constraint.W_JT1[ 0 ] * DeltaLambda[ 0 ] + Constraint.W_JT1[ 1 ] * DeltaLambda[ 1 ];
	W2 += Constraint.W_JT2[ 0 ] * DeltaLambda[ 0 ] + Constraint.W_JT2[ 1 ] * DeltaLambda[ 1 ];
	}


//--------------------------------------------------------------------------------------------------
// RkAngularConstraint3
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkAngularConstraint3& Constraint, const RkBody* Body1, const RkBody* Body2, const RkVector3& Bias, const RkVector3& Lambda )
	{
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();

	RkMatrix3 MassInv = InvI1 + InvI2;
	RkMatrix3 Mass = rkInvertT( MassInv );

	Constraint.Bias = Bias;
	Constraint.Mass = Mass;
	Constraint.Lambda = Lambda;
	Constraint.InvI1 = InvI1;
	Constraint.InvI2 = InvI2;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkAngularConstraint3& Constraint, RkVector3& W1, RkVector3& W2, const RkCompliance& Compliance )
	{
	RkVector3 RelV = W2 - W1;

	RkVector3 Lambda = Constraint.Lambda;
	RkVector3 DeltaLambda = -Compliance.MassScale * Constraint.Mass * ( RelV + Compliance.BiasScale * Constraint.Bias ) - Compliance.LambdaScale * Lambda;
	Constraint.Lambda += DeltaLambda;

	W1 -= Constraint.InvI1 * DeltaLambda;
	W2 += Constraint.InvI2 * DeltaLambda;
	}


//--------------------------------------------------------------------------------------------------
// RkSoftAngularConstraint3
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkSoftAngularConstraint3& Constraint, const RkBody* Body1, const RkBody* Body2, const RkVector3& Bias, const RkVector3& Lambda, float MaxLambda, const RkCompliance& Compliance )
	{
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();

	RkMatrix3 MassInv = InvI1 + InvI2;
	RkMatrix3 Mass = rkInvertT( MassInv );

	Constraint.BiasScale = Compliance.BiasScale;
	Constraint.Bias = Bias;
	Constraint.MassScale = Compliance.MassScale;
	Constraint.Mass = Mass; 
	Constraint.LambdaScale = Compliance.LambdaScale;
	Constraint.Lambda = Lambda;
	Constraint.MaxLambda = MaxLambda;
	
	Constraint.InvI1 = InvI1;
	Constraint.InvI2 = InvI2;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkSoftAngularConstraint3& Constraint, RkVector3& W1, RkVector3& W2 )
	{
	RkVector3 RelV = W2 - W1;

	RkVector3 Lambda = Constraint.Lambda;
	RkVector3 DeltaLambda = -Constraint.MassScale * Constraint.Mass * ( RelV + Constraint.BiasScale * Constraint.Bias ) - Constraint.LambdaScale * Lambda;
	Constraint.Lambda += DeltaLambda;

	float LambdaSq = rkDot( Constraint.Lambda, Constraint.Lambda );
	if ( LambdaSq > Constraint.MaxLambda * Constraint.MaxLambda )
		{
		Constraint.Lambda *= Constraint.MaxLambda / rkSqrt( LambdaSq );
		}
	DeltaLambda = Constraint.Lambda - Lambda;

	W1 -= Constraint.InvI1 * DeltaLambda;
	W2 += Constraint.InvI2 * DeltaLambda;
	}


//--------------------------------------------------------------------------------------------------
// RkAngularLimit
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkAngularLimit& Constraint, const RkBody* Body1, const RkBody* Body2, const RkVector3& Axis, float Bias, float Lambda, float MinLambda, float MaxLambda, const RkCompliance& Compliance )
	{
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();

	float MassInv = rkDot( Axis, ( InvI1 + InvI2 ) * Axis );
	float Mass = MassInv * MassInv > 1000.0f * RK_F32_MIN ? 1.0f / MassInv : 0.0f;

	Constraint.Axis = Axis;
	Constraint.BiasScale = Compliance.BiasScale;
	Constraint.Bias = Bias;
	Constraint.MassScale = Compliance.MassScale;
	Constraint.Mass = Mass;
	Constraint.LambdaScale = Compliance.LambdaScale;
	Constraint.Lambda = Lambda;
	Constraint.MinLambda = MinLambda;
	Constraint.MaxLambda = MaxLambda;
	
	Constraint.W_JT1 = InvI1 * Axis;
	Constraint.W_JT2 = InvI2 * Axis;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkAngularLimit& Constraint, RkVector3& W1, RkVector3& W2 )
	{
	float RelV = rkDot( Constraint.Axis, W2 - W1 );
	
	float Lambda = Constraint.Lambda;
	float DeltaLambda = -Constraint.MassScale * Constraint.Mass * ( RelV + Constraint.BiasScale * Constraint.Bias ) - Constraint.LambdaScale * Lambda;
	Constraint.Lambda = rkClamp( Lambda + DeltaLambda, Constraint.MinLambda, Constraint.MaxLambda );
	DeltaLambda = Constraint.Lambda - Lambda;

	W1 -= Constraint.W_JT1 * DeltaLambda;
	W2 += Constraint.W_JT2 * DeltaLambda;
	}






