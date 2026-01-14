//--------------------------------------------------------------------------------------------------
// vMath.inl
//
// Copyright(C) 2012 by D. Gregorius. All rights reserved.
//--------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------
// Vector operators
//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV operator*( m32 M, v32 V )
	{
	// V' = C1 * X + C2 * Y + C3 * Z;
	v32 X = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	v32 Y = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	v32 Z = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 2, 2, 2, 2 ) );

	v32 Product1 = _mm_mul_ps( M.C1, X );
	v32 Product2 = _mm_mul_ps( M.C2, Y );
	v32 Product3 = _mm_mul_ps( M.C3, Z );

	return _mm_add_ps( Product1, _mm_add_ps( Product2, Product3 ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV operator+( v32 V1, v32 V2 )
	{
	return _mm_add_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV operator-( v32 V1, v32 V2 )
	{
	return _mm_sub_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV operator*( v32 V1, v32 V2 )
	{
	return _mm_mul_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV operator/( v32 V1, v32 V2 )
	{
	return _mm_div_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV operator+( v32 V )
	{
	return V;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV operator-( v32 V )
	{
	v32 Zero = _mm_setzero_ps();
	return _mm_sub_ps( Zero, V );
	}


//-------------------------------------------------------------------------------------------------
// Vector load/store operations
//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmLoad3( const float* Src )
	{
	v32 X = _mm_load_ss( Src + 0 );
	v32 Y = _mm_load_ss( Src + 1 );
	v32 Z = _mm_load_ss( Src + 2 );
	
	v32 XY = _mm_unpacklo_ps( X, Y );
	return _mm_movelh_ps( XY, Z );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmLoad3A( const float* Src )
	{
	// Read an extra float which gets zeroed (to avoid NaN or Inf)
	v32 V = _mm_load_ps( Src );
	return _mm_and_ps( V, VM_MASK3 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmLoad4( const float* Src )
	{
	return _mm_loadu_ps( Src );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmLoad4A( const float* Src )
	{
	return _mm_load_ps( Src );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE void VM_CALLCONV vmStore3( float* Dst, v32 V )
	{
	v32 Y = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	v32 Z = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 2, 2, 2, 2 ) );
	_mm_store_ss( Dst + 0, V );
	_mm_store_ss( Dst + 1, Y );
	_mm_store_ss( Dst + 2, Z );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE void VM_CALLCONV vmStore3A( float* Dst, v32 V )
	{
	v32 Y = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	v32 Z = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 2, 2, 2, 2 ) );
	_mm_store_ss( Dst + 0, V );
	_mm_store_ss( Dst + 1, Y );
	_mm_store_ss( Dst + 2, Z );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE void VM_CALLCONV vmStore4( float* Dst, v32 V )
	{
	_mm_storeu_ps( Dst, V );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE void VM_CALLCONV vmStore4A( float* Dst, v32 V )
	{
	_mm_store_ps( Dst, V );
	}


//-------------------------------------------------------------------------------------------------
// General vector operations
//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmZero()
	{
	return _mm_setzero_ps();
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmSet( float X, float Y, float Z, float W )
	{
	return _mm_set_ps( W, Z, Y, X );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE float VM_CALLCONV vmGetX( v32 V )
	{
	return _mm_cvtss_f32( V );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE float VM_CALLCONV vmGetY( v32 V )
	{
	return _mm_cvtss_f32( _mm_shuffle_ps( V, V, _MM_SHUFFLE( 1, 1, 1, 1 ) ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE float VM_CALLCONV vmGetZ( v32 V )
	{
	return _mm_cvtss_f32( _mm_shuffle_ps( V, V, _MM_SHUFFLE( 2, 2, 2, 2 ) ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE float VM_CALLCONV vmGetW( v32 V )
	{
	return _mm_cvtss_f32( _mm_shuffle_ps( V, V, _MM_SHUFFLE( 3, 3, 3, 3 ) ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE float VM_CALLCONV vmGet( v32 V, int Index )
	{
	return V.m128_f32[ Index ];
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmSplat( float X )
	{
	return _mm_set_ps1( X );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmSplatX( v32 V )
	{
	return _mm_shuffle_ps( V, V, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmSplatY( v32 V )
	{
	return _mm_shuffle_ps( V, V, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmSplatZ( v32 V )
	{
	return _mm_shuffle_ps( V, V, _MM_SHUFFLE( 2, 2, 2, 2 ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmSplatW( v32 V )
	{
	return _mm_shuffle_ps( V, V, _MM_SHUFFLE( 3, 3, 3, 3 ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmMul( m32 M, v32 V )
	{
	// V' = C1 * X + C2 * Y + C3 * Z;
	v32 X = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	v32 Y = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	v32 Z = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 2, 2, 2, 2 ) );

	v32 Product1 = _mm_mul_ps( M.C1, X );
	v32 Product2 = _mm_mul_ps( M.C2, Y );
	v32 Product3 = _mm_mul_ps( M.C3, Z );

	return _mm_add_ps( Product1, _mm_add_ps( Product2, Product3 ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmTMul( m32 _M, v32 V )
	{
	m32 M = _M;
	VM_TRANSPOSE3( M.C1, M.C2, M.C3 );

	// V' = C1 * X + C2 * Y + C3 * Z;
	v32 X = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	v32 Y = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	v32 Z = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 2, 2, 2, 2 ) );

	v32 Product1 = _mm_mul_ps( M.C1, X );
	v32 Product2 = _mm_mul_ps( M.C2, Y );
	v32 Product3 = _mm_mul_ps( M.C3, Z );

	return _mm_add_ps( Product1, _mm_add_ps( Product2, Product3 ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmAdd( v32 V1, v32 V2 )
	{
	return _mm_add_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmSub( v32 V1, v32 V2 )
	{
	return _mm_sub_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmMul( v32 V1, v32 V2 )
	{
	return _mm_mul_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmDiv( v32 V1, v32 V2 )
	{
	return _mm_div_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmNegate( v32 V )
	{
	v32 Zero = _mm_setzero_ps();
	return _mm_sub_ps( Zero, V );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmAbs( v32 V )
	{
	// Abs( V ) = Max( -V, V )
	v32 Zero = _mm_setzero_ps();
	return _mm_max_ps( _mm_sub_ps( Zero, V ), V );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmMin( v32 V1, v32 V2 )
	{
	return _mm_min_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmMax( v32 V1, v32 V2 )
	{
	return _mm_max_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmClamp( v32 V, v32 Min, v32 Max )
	{
	return _mm_min_ps( _mm_max_ps( Min, V ), Max );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmEqual( v32 V1, v32 V2 )
	{
	return _mm_cmpeq_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmLess( v32 V1, v32 V2 )
	{
	return _mm_cmplt_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmLessEq( v32 V1, v32 V2 )
	{
	return _mm_cmple_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmGreater( v32 V1, v32 V2 )
	{
	return _mm_cmpgt_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmGreaterEq( v32 V1, v32 V2 )
	{
	return _mm_cmpge_ps( V1, V2 );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmSelect( v32 C, v32 V1, v32 V2 )
	{
	// return C[i] == 0 ? V1[i] : V2[i]
	v32 Conditional1 = _mm_andnot_ps( C, V1 );
	v32 Conditional2 = _mm_and_ps( V2, C );
	return _mm_or_ps( Conditional1, Conditional2 );
	}


//-------------------------------------------------------------------------------------------------
// 3d vector operations
//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmCross3( v32 V1, v32 V2 )
	{
	v32 YZX1 = _mm_shuffle_ps( V1, V1, _MM_SHUFFLE( 3, 0, 2, 1 ) );
	v32 ZXY1 = _mm_shuffle_ps( V1, V1, _MM_SHUFFLE( 3, 1, 0, 2 ) );
	v32 YZX2 = _mm_shuffle_ps( V2, V2, _MM_SHUFFLE( 3, 0, 2, 1 ) );
	v32 ZXY2 = _mm_shuffle_ps( V2, V2, _MM_SHUFFLE( 3, 1, 0, 2 ) );

	return _mm_sub_ps( _mm_mul_ps( YZX1, ZXY2 ), _mm_mul_ps( ZXY1, YZX2 ) );
	}


//--------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmModifiedCross3( v32 V1, v32 V2 )
	{
	v32 YZX1 = _mm_shuffle_ps( V1, V1, _MM_SHUFFLE( 3, 0, 2, 1 ) );
	v32 ZXY1 = _mm_shuffle_ps( V1, V1, _MM_SHUFFLE( 3, 1, 0, 2 ) );
	v32 YZX2 = _mm_shuffle_ps( V2, V2, _MM_SHUFFLE( 3, 0, 2, 1 ) );
	v32 ZXY2 = _mm_shuffle_ps( V2, V2, _MM_SHUFFLE( 3, 1, 0, 2 ) );

	return _mm_add_ps( _mm_mul_ps( YZX1, ZXY2 ), _mm_mul_ps( ZXY1, YZX2 ) );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmDot3( v32 V1, v32 V2 )
	{
	v32 M = _mm_mul_ps( V1, V2 );
	v32 X = _mm_shuffle_ps( M, M, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	v32 Y = _mm_shuffle_ps( M, M, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	v32 Z = _mm_shuffle_ps( M, M, _MM_SHUFFLE( 2, 2, 2, 2 ) );

	return _mm_add_ps( _mm_add_ps( X, Y ), Z );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmLength3( v32 V )
	{
	v32 M = _mm_mul_ps( V, V );
	v32 X = _mm_shuffle_ps( M, M, _MM_SHUFFLE( 0, 0, 0, 0 ) );
	v32 Y = _mm_shuffle_ps( M, M, _MM_SHUFFLE( 1, 1, 1, 1 ) );
	v32 Z = _mm_shuffle_ps( M, M, _MM_SHUFFLE( 2, 2, 2, 2 ) );
	v32 Dot = _mm_add_ps( _mm_add_ps( X, Y ), Z );

	return _mm_sqrt_ps( Dot );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE v32 VM_CALLCONV vmNormalize3( v32 V )
	{
	v32 Length = vmLength3( V );
	v32 Zero = vmZero();
	
	v32 Conditional = vmEqual( Length, Zero );
	return vmSelect( Conditional, vmDiv( V, Length ), Zero );
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAnyEqual3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmpeq_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) != 0;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAnyLess3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmplt_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) != 0;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAnyLessEq3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmple_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) != 0;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAnyGreater3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmpgt_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) != 0;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAnyGreaterEq3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmpge_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) != 0;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAllEqual3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmpeq_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) == 0x07;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAllLess3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmplt_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) == 0x07;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAllLessEq3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmple_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) == 0x07;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAllGreater3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmpgt_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) == 0x07;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmAllGreaterEq3( v32 V1, v32 V2 )
	{
	v32 V = _mm_cmpge_ps( V1, V2 );
	return ( _mm_movemask_ps( V ) & 0x07 ) == 0x07;
	}


//-------------------------------------------------------------------------------------------------
// Matrix type
//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE m32 VM_CALLCONV operator+( m32 M1, m32 M2 )
	{
	m32 Out;
	Out.C1 = _mm_add_ps( M1.C1, M2.C1 );
	Out.C2 = _mm_add_ps( M1.C2, M2.C2 );
	Out.C3 = _mm_add_ps( M1.C3, M2.C3 );

	return Out;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE m32 VM_CALLCONV operator-( m32 M1, m32 M2 )
	{
	m32 Out;
	Out.C1 = _mm_sub_ps( M1.C1, M2.C1 );
	Out.C2 = _mm_sub_ps( M1.C2, M2.C2 );
	Out.C3 = _mm_sub_ps( M1.C3, M2.C3 );

	return Out;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE m32 VM_CALLCONV operator*( m32 M1, m32 M2 )
	{
	m32 Out;
	Out.C1 = M1 * M2.C1;
	Out.C2 = M1 * M2.C2;
	Out.C3 = M1 * M2.C3;

	return Out;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE m32 VM_CALLCONV operator+( m32 M )
	{
	return M;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE m32 VM_CALLCONV operator-( m32 M )
	{
	v32 Zero = _mm_setzero_ps();

	m32 Out;
	Out.C1 = _mm_sub_ps( Zero, M.C1 );
	Out.C2 = _mm_sub_ps( Zero, M.C2 );
	Out.C3 = _mm_sub_ps( Zero, M.C3 );

	return Out;
	}


//-------------------------------------------------------------------------------------------------
// 3d matrix operations
//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE m32 VM_CALLCONV vmTranspose( m32 M )
	{
	v32 Lo = _mm_unpacklo_ps( ( M.C1 ), ( M.C2 ) );								
	v32 Hi = _mm_unpackhi_ps( ( M.C1 ), ( M.C2 ) );	
	
	m32 Out;
	Out.C1 = _mm_shuffle_ps( ( Lo ), ( M.C3 ), _MM_SHUFFLE( 0, 0, 1, 0 ) );
	Out.C2 = _mm_shuffle_ps( ( Lo ), ( M.C3 ), _MM_SHUFFLE( 1, 1, 3, 2 ) );
	Out.C3 = _mm_shuffle_ps( ( Hi ), ( M.C3 ), _MM_SHUFFLE( 2, 2, 1, 0 ) );

	return Out;
	}
	

//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE m32 VM_CALLCONV vmAbs( m32 M )
	{
	// Abs( V ) = Max( -V, V )
	v32 Zero = _mm_setzero_ps();

	m32 Out;
	Out.C1 = _mm_max_ps( _mm_sub_ps( Zero, M.C1 ), M.C1 );
	Out.C2 = _mm_max_ps( _mm_sub_ps( Zero, M.C2 ), M.C2 );
	Out.C3 = _mm_max_ps( _mm_sub_ps( Zero, M.C3 ), M.C3 );

	return Out;
	}


//-------------------------------------------------------------------------------------------------
VM_FORCEINLINE m32 VM_CALLCONV vmSkew( v32 V )
	{
	// C1 = (  0,  Z, -Y, 0 )
	// C2 = ( -Z,  0,  X, 0 )
	// C3 = (  Y, -X,  0, 0 )
	v32 ZZYY = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 1, 1, 2, 2 ) );
	v32 ZZXX = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 0, 0, 2, 2 ) );
	v32 YXYX = _mm_shuffle_ps( V, V, _MM_SHUFFLE( 0, 1, 0, 1 ) );

	m32 Out;
	Out.C1 = _mm_mul_ps( ZZYY, VM_CROSS1 );
	Out.C2 = _mm_mul_ps( ZZXX, VM_CROSS2 );
	Out.C3 = _mm_mul_ps( YXYX, VM_CROSS3 );

	return Out;
	}


//--------------------------------------------------------------------------------------------------
// AABB overlap tests
//--------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmTestBoundsOverlap( v32 NodeMin1, v32 NodeMax1, v32 NodeMin2, v32 NodeMax2 )
	{
	v32 Separation = vmMax( NodeMin2 - NodeMax1, NodeMin1 - NodeMax2 );
	return vmAllLessEq3( Separation, VM_ZERO );
	}


//--------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmTestBoundsRayOverlap( v32 NodeMin, v32 NodeMax, v32 RayStart, v32 RayDelta )
	{
	// Setup node 
	v32 NodeCenter = VM_HALF * ( NodeMin + NodeMax );
	v32 NodeExtent = NodeMax - NodeCenter;

	// Setup ray
	RayStart = RayStart - NodeCenter;
	
	// SAT: Edge separation
	v32 EdgeSeparation = vmAbs( vmCross3( RayDelta, RayStart ) ) - vmModifiedCross3( vmAbs( RayDelta ), NodeExtent );
	return vmAllLessEq3( EdgeSeparation, VM_ZERO );
	}


//--------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmTestBoundsRayOverlap( v32 NodeMin, v32 NodeMax, v32 RayStart, v32 RayDelta, v32 Lambda )
	{
	v32 NodeCenter = VM_HALF * ( NodeMin + NodeMax );
	v32 NodeExtent = NodeMax - NodeCenter;

	// Setup ray
	RayStart = RayStart - NodeCenter;
	v32 RayEnd = RayStart + Lambda * RayDelta;
	v32 RayMin = vmMin( RayStart, RayEnd );
	v32 RayMax = vmMax( RayStart, RayEnd );

	// SAT: Face separation
	v32 Separation1 = RayMin - NodeExtent;
	v32 Separation2 = RayMax + NodeExtent;
	v32 FaceSeparation = vmMax( Separation1, -Separation2 );
	if ( vmAnyGreater3( FaceSeparation, VM_ZERO ) )
		{
		return false;
		}

	// SAT: Edge separation
	v32 EdgeSeparation = vmAbs( vmCross3( RayDelta, RayStart ) ) - vmModifiedCross3( vmAbs( RayDelta ), NodeExtent );
	return vmAllLessEq3( EdgeSeparation, VM_ZERO );
	}


//--------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmTestBoundsTriangleOverlap( v32 NodeCenter, v32 NodeExtent, v32 Vertex1, v32 Vertex2, v32 Vertex3 )
	{
	// Setup triangle
	Vertex1 = Vertex1 - NodeCenter;
	Vertex2 = Vertex2 - NodeCenter;
	Vertex3 = Vertex3 - NodeCenter;

	// Face separation
	v32 TriangleMin = vmMin( Vertex1, vmMin( Vertex2, Vertex3 ) );
	v32 TriangleMax = vmMax( Vertex1, vmMax( Vertex2, Vertex3 ) );

	v32 Separation1 = TriangleMin - NodeExtent;
	v32 Separation2 = TriangleMax + NodeExtent;

	v32 FaceSeparation = vmMax( Separation1, -Separation2 );
	if ( vmAnyGreater3( FaceSeparation, VM_ZERO ) )
		{
		return false;
		}

	// SAT: Face separation
	v32 Edge1 = Vertex2 - Vertex1;
	v32 Edge2 = Vertex3 - Vertex2;
	v32 Edge3 = Vertex1 - Vertex3;

	v32 Normal = vmCross3( Edge1, Edge2 );

	v32 TriangleSeparation = vmAbs( vmDot3( Normal, Vertex1 ) ) - vmDot3( vmAbs( Normal ), NodeExtent );
	if ( vmAnyGreater3( TriangleSeparation, VM_ZERO ) )
		{
		return false;
		}

	// SAT: Edge separation
	v32 EdgeSeparation1 = vmAbs( vmCross3( Edge1, Vertex1 + Vertex3 ) ) - vmAbs( vmCross3( Edge1, Edge3 ) ) - VM_TWO * vmModifiedCross3( vmAbs( Edge1 ), NodeExtent );
	if ( vmAnyGreater3( EdgeSeparation1, VM_ZERO ) )
		{
		return false;
		}

	v32 EdgeSeparation2 = vmAbs( vmCross3( Edge2, Vertex1 + Vertex2 ) ) - vmAbs( vmCross3( Edge2, Edge1 ) ) - VM_TWO * vmModifiedCross3( vmAbs( Edge2 ), NodeExtent );
	if ( vmAnyGreater3( EdgeSeparation2, VM_ZERO ) )
		{
		return false;
		}

	v32 EdgeSeparation3 = vmAbs( vmCross3( Edge3, Vertex2 + Vertex3 ) ) - vmAbs( vmCross3( Edge3, Edge2 ) ) - VM_TWO * vmModifiedCross3( vmAbs( Edge3 ), NodeExtent );
	if ( vmAnyGreater3( EdgeSeparation3, VM_ZERO ) )
		{
		return false;
		}

	return true;
	}


//--------------------------------------------------------------------------------------------------
VM_FORCEINLINE bool VM_CALLCONV vmContains( v32 ParentMin, v32 ParentMax, v32 ChildMin, v32 ChildMax )
	{
	v32 CmpMin = _mm_cmpge_ps( ChildMin, ParentMin );
	v32 CmpMax = _mm_cmple_ps( ChildMax, ParentMax );
	v32 Result = _mm_and_ps( CmpMin, CmpMax );

	return ( _mm_movemask_ps( Result ) & 0x07 ) == 0x07;
	}


//--------------------------------------------------------------------------------------------------
VM_FORCEINLINE void VM_CALLCONV vmUnion( v32& VM_RESTRICT ParentMin, v32& VM_RESTRICT ParentMax, v32 ChildMin, v32 ChildMax )
	{
	ParentMin = _mm_min_ps( ParentMin, ChildMin );
	ParentMax = _mm_max_ps( ParentMax, ChildMax );
	}


//--------------------------------------------------------------------------------------------------
// Ray intersection tests
//--------------------------------------------------------------------------------------------------
VM_FORCEINLINE float VM_CALLCONV vmIntersectRayTriangle( v32 RayStart, v32 RayDelta, v32 Vertex1, v32 Vertex2, v32 Vertex3 )
	{
	// Test if ray intersects this triangle sharing same calculations for each triangle
		{
		v32 Edge1 = Vertex3 - Vertex2;
		v32 Edge2 = Vertex1 - Vertex3;
		v32 Edge3 = Vertex2 - Vertex1;

		v32 MidPoint1 = VM_HALF * ( Vertex2 + Vertex3 );
		v32 MidPoint2 = VM_HALF * ( Vertex3 + Vertex1 );
		v32 MidPoint3 = VM_HALF * ( Vertex1 + Vertex2 );

		v32 Normal1 = vmCross3( Edge1, MidPoint1 - RayStart );
		v32 Normal2 = vmCross3( Edge2, MidPoint2 - RayStart );
		v32 Normal3 = vmCross3( Edge3, MidPoint3 - RayStart );
		VM_TRANSPOSE3( Normal1, Normal2, Normal3 );
		
		v32 RayDeltaX = vmSplatX( RayDelta );
		v32 RayDeltaY = vmSplatY( RayDelta );
		v32 RayDeltaZ = vmSplatZ( RayDelta );
		
		v32 Volumes = Normal1 * RayDeltaX + Normal2 * RayDeltaY + Normal3 * RayDeltaZ;
		if ( vmAnyLess3( Volumes, VM_ZERO ) )
			{
			return 1.0f;
			}
		}

	// Compute intersection with triangle plane
	v32 Edge1 = Vertex2 - Vertex1;
	v32 Edge2 = Vertex3 - Vertex1;
	v32 Normal = vmCross3( Edge1, Edge2 );

	v32 Denominator = vmDot3( Normal, RayDelta );
	if ( vmAnyGreaterEq3( Denominator, VM_ZERO ) )
		{
		return 1.0f;
		}
		
	v32 Lambda = vmDot3( Normal, Vertex1 - RayStart ) / Denominator;
	if ( vmAnyLessEq3( Lambda, VM_ZERO ) )
		{
		return 1.0f;
		}

	Lambda = vmMin( Lambda, VM_ONE );
	return vmGetX( Lambda );
	}







