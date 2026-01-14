//--------------------------------------------------------------------------------------------------
// contactsolver.inl	
//
// Copyright(C) by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
// RkAngularFrictionConstraint
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkAngularFrictionConstraint& Out, const RkBody* Body1, const RkBody* Body2, const RkVector3& Normal, float Lambda, float MaxLambda )
	{
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();

	float MassInv = rkDot( Normal, ( InvI1 + InvI2 ) * Normal );
	float Mass = MassInv * MassInv > 1000.0f * RK_F32_MIN ? 1.0f / MassInv : 0.0f;

	Out.Normal = Normal;
	Out.Mass = Mass;
	Out.Lambda = Lambda;
	Out.MaxLambda = MaxLambda;
	Out.W_JT1 = InvI1 * Normal;
	Out.W_JT2 = InvI2 * Normal;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkWarmstartConstraint( const RkAngularFrictionConstraint& Constraint, RkVector3& W1, RkVector3& W2 )
	{
	W1 -= Constraint.W_JT1 * Constraint.Lambda;
	W2 += Constraint.W_JT2 * Constraint.Lambda;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkAngularFrictionConstraint& Constraint, RkVector3& W1, RkVector3& W2 )
	{
	float RelV = rkDot( Constraint.Normal, W2 - W1 );
	
	float Lambda = Constraint.Lambda;
	float DeltaLambda = Constraint.Mass * -RelV;
	Constraint.Lambda = rkClamp( Lambda + DeltaLambda, -Constraint.MaxLambda, Constraint.MaxLambda );
	DeltaLambda = Constraint.Lambda - Lambda;

	W1 -= Constraint.W_JT1 * DeltaLambda;
	W2 += Constraint.W_JT2 * DeltaLambda;
	}


//--------------------------------------------------------------------------------------------------
// RkLinearFrictionConstraint
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkLinearFrictionConstraint& Out, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, RkVector3 Tangent[ 2 ], RkVector2 Lambda, float MaxLambda )
	{
	float InvM1 = Body1->GetMassInv();
    RkMatrix3 InvI1 = Body1->GetInertiaInv();
	float InvM2 = Body2->GetMassInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();

	RkVector3 R1_x_S = rkCross( Offset1, Tangent[ 0 ] );
	RkVector3 R2_x_S = rkCross( Offset2, Tangent[ 0 ] );
	RkVector3 R1_x_T = rkCross( Offset1, Tangent[ 1 ] );
	RkVector3 R2_x_T = rkCross( Offset2, Tangent[ 1 ] );

	RkMatrix2 K;
	K.A11 = InvM1 + InvM2 + rkDot( R1_x_S, InvI1 * R1_x_S ) + rkDot( R2_x_S, InvI2 * R2_x_S );
	K.A22 = InvM1 + InvM2 + rkDot( R1_x_T, InvI1 * R1_x_T ) + rkDot( R2_x_T, InvI2 * R2_x_T );
	K.A12 = K.A21 = rkDot( R1_x_S, InvI1 * R1_x_T ) + rkDot( R2_x_S, InvI2 * R2_x_T );

	Out.Tangent[ 0 ] = Tangent[ 0 ];
	Out.Tangent[ 1 ] = Tangent[ 1 ];
	Out.Offset1 = Offset1;
	Out.Offset2 = Offset2;
	Out.Mass = rkInvertT( K );
	Out.Lambda = Lambda;
	Out.MaxLambda = MaxLambda;
	Out.InvM1 = InvM1;
	Out.W_JT1[ 0 ] = InvI1 * R1_x_S;
	Out.W_JT1[ 1 ] = InvI1 * R1_x_T;
	Out.InvM2 = InvM2;
	Out.W_JT2[ 0 ] = InvI2 * R2_x_S;
	Out.W_JT2[ 1 ] = InvI2 * R2_x_T;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkWarmstartConstraint( const RkLinearFrictionConstraint& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 )
	{
	RkVector3 Impulse = Constraint.Lambda[ 0 ] * Constraint.Tangent[ 0 ] + Constraint.Lambda[ 1 ] * Constraint.Tangent[ 1 ];

	V1 -= Constraint.InvM1 * Impulse;
	W1 -= Constraint.W_JT1[ 0 ] * Constraint.Lambda[ 0 ] + Constraint.W_JT1[ 1 ] * Constraint.Lambda[ 1 ];
	V2 += Constraint.InvM2 * Impulse;
	W2 += Constraint.W_JT2[ 0 ] * Constraint.Lambda[ 0 ] + Constraint.W_JT2[ 1 ] * Constraint.Lambda[ 1 ];
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkLinearFrictionConstraint& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 )
	{
	RkVector3 U1 = V1 + rkCross( W1, Constraint.Offset1 );
	RkVector3 U2 = V2 + rkCross( W2, Constraint.Offset2 );
	RkVector2 RelV = RkVector2( rkDot( U2 - U1, Constraint.Tangent[ 0 ] ), rkDot( U2 - U1, Constraint.Tangent[ 1 ] ) );
	
	RkVector2 Lambda = Constraint.Lambda;
	RkVector2 DeltaLambda = Constraint.Mass * -RelV;
	Constraint.Lambda += DeltaLambda;
	float LambdaSq = rkLengthSq( Constraint.Lambda ); 
	if ( LambdaSq > Constraint.MaxLambda * Constraint.MaxLambda )
		{
		Constraint.Lambda *= Constraint.MaxLambda / rkSqrt( LambdaSq );
		}
	DeltaLambda = Constraint.Lambda - Lambda;

	RkVector3 Impulse = DeltaLambda[ 0 ] * Constraint.Tangent[ 0 ] + DeltaLambda[ 1 ] * Constraint.Tangent[ 1 ];

	V1 -= Constraint.InvM1 * Impulse; 
	W1 -= Constraint.W_JT1[ 0 ] * DeltaLambda[ 0 ] + Constraint.W_JT1[ 1 ] * DeltaLambda[ 1 ];
	V2 += Constraint.InvM2 * Impulse;
	W2 += Constraint.W_JT2[ 0 ] * DeltaLambda[ 0 ] + Constraint.W_JT2[ 1 ] * DeltaLambda[ 1 ];
	}


//--------------------------------------------------------------------------------------------------
// RkNonPenetrationConstraint
//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkCreateConstraint( RkNonPenetrationConstraint& Out, const RkBody* Body1, const RkVector3& Offset1, const RkBody* Body2, const RkVector3& Offset2, const RkVector3& Normal, float Bias, float Lambda )
	{
	float InvM1 = Body1->GetMassInv();
	RkMatrix3 InvI1 = Body1->GetInertiaInv();
	float InvM2 = Body2->GetMassInv();
	RkMatrix3 InvI2 = Body2->GetInertiaInv();

	RkVector3 R1_x_N = rkCross( Offset1, Normal );
	RkVector3 R2_x_N = rkCross( Offset2, Normal );

	float MassInv = InvM1 + InvM2 + rkDot( R1_x_N, InvI1 * R1_x_N ) + rkDot( R2_x_N, InvI2 * R2_x_N );
	float Mass = MassInv * MassInv > 1000.0f * RK_F32_MIN ? 1.0f / MassInv : 0.0f;

	Out.Normal = Normal;
	Out.Offset1 = Offset1;
	Out.Offset2 = Offset2;
	Out.Bias = Bias;
	Out.Mass = Mass;
	Out.Lambda = Lambda;
	Out.InvM1 = InvM1;
	Out.W_JT1 = InvI1 * R1_x_N;
	Out.InvM2 = InvM2;
	Out.W_JT2 = InvI2 * R2_x_N;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkWarmstartConstraint( const RkNonPenetrationConstraint& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2 )
	{
	RkVector3 Impulse = Constraint.Lambda * Constraint.Normal;
	
	V1 -= Constraint.InvM1 * Impulse;
	W1 -= Constraint.W_JT1 * Constraint.Lambda;
	V2 += Constraint.InvM2 * Impulse;
	W2 += Constraint.W_JT2 * Constraint.Lambda;
	}


//--------------------------------------------------------------------------------------------------
RK_FORCEINLINE void rkSolveConstraint( RkNonPenetrationConstraint& Constraint, RkVector3& V1, RkVector3& W1, RkVector3& V2, RkVector3& W2, const RkCompliance& Compliance )
	{
	RkVector3 U1 = V1 + rkCross( W1, Constraint.Offset1 );
	RkVector3 U2 = V2 + rkCross( W2, Constraint.Offset2 );
	float RelV = rkDot( Constraint.Normal, U2 - U1 );

	float Lambda = Constraint.Lambda;
	float DeltaLambda = -Compliance.MassScale * Constraint.Mass * ( RelV + Compliance.BiasScale * Constraint.Bias ) - Compliance.LambdaScale * Lambda;
	Constraint.Lambda = rkMax( 0.0f, Lambda + DeltaLambda );
	DeltaLambda = Constraint.Lambda - Lambda;

	RkVector3 Impulse = DeltaLambda * Constraint.Normal;

	V1 -= Constraint.InvM1 * Impulse;
	W1 -= Constraint.W_JT1 * DeltaLambda;
	V2 += Constraint.InvM2 * Impulse;
	W2 += Constraint.W_JT2 * DeltaLambda;
	}